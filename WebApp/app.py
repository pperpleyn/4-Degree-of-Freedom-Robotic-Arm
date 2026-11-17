"""Robotic arm controller using the provided app.ui and MyTelemetrix.

Keys: WASD wheels; I/K arm (servo 2); U/J elbow (servo 1);
N/M pivot (servo 0); Space claw toggle (servo 3).
"""
import sys
from pathlib import Path
import urllib.request
import time
import threading

from PySide6.QtCore import QEvent, QFile, QIODevice, QObject, Qt, QUrl, Signal, Slot, QThread, QTimer, QRunnable, QThreadPool
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QApplication, QLabel, QLineEdit, QPushButton, QSlider, QStackedLayout, QWidget, QSizePolicy, QStyle, QStyleOptionSlider
from PySide6.QtGui import QImage, QPixmap, QIntValidator

from utils.myTelemetrix import MyTelemetrix

PORT_NAME = "COM5"   # Change to your Bluetooth COM port; no connection is attempted at startup.
STREAM_URL = "http://192.168.4.1:81/stream"  
#http://10.49.174.193:81/stream
SERVO_STEP = 5
CONTROL_CENTER = 90
SERVO_INTERVAL_MS = 50
SERVO_DEGREES_PER_SECOND = 30
CONTROL_DEADZONE = 3
GRIP_CLOSED = 80
GRIP_OPEN = 10
NEUTRAL = 49


class MotorValue:
    """Value-only adapter for the original MyTelemetrix.update_motors(slider, slider)."""
    def __init__(self, value=NEUTRAL):
        self.position = value

    def value(self):
        return self.position


class CommandSignals(QObject):
    finished = Signal(object)


class BluetoothCommand(QRunnable):
    """One blocking hardware operation. Arguments contain only plain values."""
    def __init__(self, board, action, args):
        super().__init__()
        self.board, self.action, self.args = board, action, args
        self.signals = CommandSignals()

    @Slot()
    def run(self):
        board = self.board
        error = ""
        try:
            if self.action == "connect":
                port, angles, speed = self.args
                if not board.reconn(port):
                    raise RuntimeError(board.last_error or "Bluetooth connection failed")
                board.stop_motors()
                board.set_motor_speed(speed)
                for channel, angle in angles:
                    board.set_servo_angle(channel, angle)
            elif self.action == "disconnect":
                try:
                    if board.connected:
                        board.stop_motors()
                finally:
                    board.close()
            elif self.action == "speed":
                board.set_motor_speed(self.args[0])
            elif board.connected:
                if self.action == "servo":
                    board.set_servo_angle(*self.args)
                elif self.action == "motor":
                    forward, turn = self.args
                    board.update_motors(MotorValue(forward), MotorValue(turn))
            if self.action in ("connect", "servo", "motor") and not board.connected:
                raise RuntimeError(board.last_error or "Bluetooth disconnected")
        except Exception as exc:
            error = str(exc)
            try:
                board.close()
            except Exception as close_error:
                error += f"; close failed: {close_error}"
        self.signals.finished.emit({
            "action": self.action,
            "connected": bool(board.connected) and not error,
            "port": board.com_port,
            "error": error or board.last_error or "",
        })


class BluetoothPool(QObject):
    """Serialize board access and keep the command queue small."""
    state_changed = Signal(object)

    def __init__(self, board, parent):
        super().__init__(parent)
        self.board = board
        self.pool = QThreadPool(self)
        self.pool.setMaxThreadCount(1)
        self.pool.setExpiryTimeout(-1)
        self.pending = []
        self.active = None

    def submit(self, action, *args):
        key = (action, args[0]) if action == "servo" else (action,)
        if action in ("connect", "disconnect"):
            self.pending.clear()
        else:
            # Keep a pending stop before a later press.
            self.pending = [
                item for item in self.pending
                if item[0] != key or (item[1] == "motor" and item[2] == (NEUTRAL, NEUTRAL))
            ]
        entry = (key, action, args)
        if action == "motor" and args == (NEUTRAL, NEUTRAL):
            self.pending = [item for item in self.pending if item[1] != "motor"]
            self.pending.insert(0, entry)
        else:
            self.pending.append(entry)
        self.start_next()

    def start_next(self):
        if self.active is not None or not self.pending:
            return
        _, action, args = self.pending.pop(0)
        self.active = BluetoothCommand(self.board, action, args)
        self.active.signals.finished.connect(self.completed, Qt.ConnectionType.QueuedConnection)
        self.pool.start(self.active)

    @Slot(object)
    def completed(self, result):
        self.active = None
        if result["error"] and not result["connected"]:
            # Retain an explicit lifecycle operation such as shutdown or retry.
            self.pending = [item for item in self.pending if item[1] in ("connect", "disconnect")]
        self.state_changed.emit(result)
        self.start_next()

    def idle(self):
        return self.active is None and not self.pending and self.pool.activeThreadCount() == 0


class KeyFilter(QObject):
    drive_keys = {Qt.Key.Key_W: "upBtn", Qt.Key.Key_S: "downBtn",
                  Qt.Key.Key_A: "leftBtn", Qt.Key.Key_D: "rightBtn"}
    servo_keys = {
        Qt.Key.Key_I: (2, 1), Qt.Key.Key_K: (2, -1),
        Qt.Key.Key_U: (1, 1), Qt.Key.Key_J: (1, -1),
        Qt.Key.Key_N: (0, 1), Qt.Key.Key_M: (0, -1),
    }

    def __init__(self, owner):
        super().__init__(owner.window)
        self.owner = owner
        self.held = []
        self.joint_held = []
        self.mouse_channel = None

    def eventFilter(self, watched, event):
        owner = self.owner
        kind = event.type()
        if watched is owner.window:
            if kind == QEvent.Type.Close and not owner.close_ready:
                event.ignore()
                owner.shutdown()
                return True
            if kind == QEvent.Type.Resize:
                owner.resize_layout()
                return False
            if kind == QEvent.Type.WindowDeactivate:
                self.clear()
                return False
        # Handle mouse input ourselves: no native groove auto-repeat/overshoot.
        channel = next((c for c, s in owner.sliders.items() if watched is s), None)
        if channel is not None:
            if kind == QEvent.Type.MouseButtonPress and event.button() == Qt.MouseButton.LeftButton:
                if not owner.closing and not owner.connection_busy:
                    self.mouse_channel = channel
                    owner.mouse_control(channel, event.position().toPoint())
                return True
            if kind == QEvent.Type.MouseMove and self.mouse_channel == channel:
                owner.mouse_control(channel, event.position().toPoint())
                return True
            if kind == QEvent.Type.MouseButtonRelease and event.button() == Qt.MouseButton.LeftButton:
                self.mouse_channel = None
                self.apply_joint(channel)
                return True
            if kind == QEvent.Type.Wheel:
                return True
        if kind not in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            return False
        if not isinstance(watched, QWidget) or (watched is not owner.window and
                not owner.window.isAncestorOf(watched)):
            return False
        key = event.key()
        pressed = kind == QEvent.Type.KeyPress
        if key not in self.drive_keys and key not in self.servo_keys and key != Qt.Key.Key_Space:
            return False
        # Releases are processed even if a text box gained focus.
        if pressed and (owner.closing or owner.connection_busy or
                        isinstance(owner.window.focusWidget(), QLineEdit)):
            return False
        if event.isAutoRepeat():
            return True  # QTimer controls movement, not the OS repeat rate.
        if key in self.drive_keys:
            if pressed and key not in self.held:
                self.held.append(key)
            elif not pressed and key in self.held:
                self.held.remove(key)
            owner.set_keyboard_drive(self.drive_keys[self.held[-1]] if self.held else None)
        elif key in self.servo_keys:
            if pressed and key not in self.joint_held:
                self.joint_held.append(key)
            elif not pressed and key in self.joint_held:
                self.joint_held.remove(key)
            self.apply_joint(self.servo_keys[key][0])
        elif pressed:
            owner.toggle_grip()
        return True

    def apply_joint(self, channel):
        if self.mouse_channel == channel:
            return
        keys = [key for key in self.joint_held if self.servo_keys[key][0] == channel]
        sign = self.servo_keys[keys[-1]][1] if keys else 0
        self.owner.sliders[channel].setValue(CONTROL_CENTER + sign * CONTROL_CENTER)

    def clear(self):
        self.held.clear()
        self.joint_held.clear()
        self.mouse_channel = None
        for slider in self.owner.sliders.values():
            slider.setValue(CONTROL_CENTER)
        self.owner.set_keyboard_drive(None)


class RobotUI(QObject):
    reference_width = 1366
    reference_height = 768
    reference_geometries = {
        "Feed": (312, 113, 620, 260),
        "feedTitle": (312, 68, 620, 36),
        "refreshFeedBtn": (547, 390, 150, 42),
        "armTitle": (960, 96, 80, 40),
        "elbowTitle": (1035, 96, 105, 40),
        "armSlider": (972, 145, 54, 280),
        "elbowSlider": (1055, 145, 54, 280),
        "turnTitle": (905, 425, 275, 28),
        "turnSlider": (910, 450, 270, 64),
        "upBtn": (132, 502, 56, 56),
        "leftBtn": (50, 574, 56, 56),
        "rightBtn": (214, 574, 56, 56),
        "downBtn": (132, 646, 56, 56),
        "wheelSpeedTitle": (290, 640, 197, 28),
        "wheelSpeedInput": (410, 640, 72, 28),
        "portNameEdit": (410, 460, 360, 56),
        "reconnBtn": (797, 458, 54, 60),
        "bluetoothLabel": (410, 529, 180, 35),
        "connectionStatus": (410, 570, 441, 28),
        "cameraStatus": (312, 438, 620, 20),
        "gripStatus": (0, 0, 1, 1),
        "gripBtn": (982, 542, 103, 103),
        "armValue": (814, 713, 145, 34),
        "elbowValue": (972, 713, 160, 34),
        "turnStatus": (1158, 713, 145, 34),
        }
    def __init__(self, app, board):
        super().__init__(app)
        self.application = app
        self.connected = bool(board.connected)
        self.connected_port = board.com_port
        self.last_error = board.last_error
        self.motor_speed = board.motor_speed
        self.connection_busy = False
        self.close_ready = False
        self.bluetooth = BluetoothPool(board, self)
        self.bluetooth.state_changed.connect(self.on_hardware_result)
        self.forward = MotorValue()
        self.turn = MotorValue()
        self.keyboard_drive = None
        self.mouse_drive = None
        self.grip_open = False
        self.closing = False

        base = Path(__file__).resolve().parent
        ui = QFile(str(base / "app.ui"))
        if not ui.open(QIODevice.OpenModeFlag.ReadOnly):
            raise RuntimeError(f"Could not open {ui.fileName()}")
        try:
            self.window = QUiLoader().load(ui)
        finally:
            ui.close()
        if self.window is None:
            raise RuntimeError("Could not load app.ui")
        self.window.setWindowTitle("4-DOF Robotic Arm Controller")
        self.window.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.window.setMinimumSize(950, 570)
        self.reference_size = (self.window.width(), self.window.height())
        self.rectangles = {}
        for widget in self.window.findChildren(QWidget):
            if widget.parentWidget() is self.window:
                self.rectangles[widget.objectName()] = widget.geometry()

        self.arm = self.get(QSlider, "armSlider")    # Servo 2
        self.elbow = self.get(QSlider, "elbowSlider")  # Servo 1
        self.pivot = self.get(QSlider, "turnSlider")  # Servo 0
        self.grip = self.get(QPushButton, "gripBtn")  # Servo 3 (toggle only)
        self.port = self.get(QLineEdit, "portNameEdit")
        self.status = self.get(QLabel, "connectionStatus")
        self.reconnect_button = self.get(QPushButton, "reconnBtn")
        self.arm_label = self.get(QLabel, "armValue")
        self.elbow_label = self.get(QLabel, "elbowValue")
        self.pivot_label = self.get(QLabel, "turnStatus")
        self.grip_label = self.get(QLabel, "gripStatus")
        self.feed = self.get(QWidget, "Feed")
        self.feed_placeholder = self.get(QLabel, "feedPlaceholder")
        self.camera_status = self.get(QLabel, "cameraStatus")
        self.get(QLabel, "turnTitle").setText("PIVOT (N / M)")
        self.get(QLabel, "armTitle").setText("ARM (I/K)")
        self.get(QLabel, "elbowTitle").setText("ELBOW (U/J)")
        self.get(QLabel, "feedTitle").setText("ROBOT CAMERA  •  W A S D = WHEELS")
        self.port.setText(PORT_NAME)
        self.port.setPlaceholderText("Bluetooth COM port (e.g. COM5)")
        for slider in (self.arm, self.elbow, self.pivot):
            slider.setRange(0, 180)
            slider.setSingleStep(SERVO_STEP)
            slider.setPageStep(SERVO_STEP)
            slider.setValue(90)
            slider.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.sliders = {0: self.pivot, 1: self.elbow, 2: self.arm}
        # Sliders are direction controls. These are the retained target angles.
        self.servo_angles = {0: 90.0, 1: 90.0, 2: 90.0}
        self.servo_timer = QTimer(self)
        self.servo_timer.setInterval(SERVO_INTERVAL_MS)
        self.servo_timer.timeout.connect(self.advance_servos)
        self.servo_timer.start()
        self.grip.clicked.connect(self.toggle_grip)
        self.reconnect_button.clicked.connect(self.toggle_connection)
        self.port.returnPressed.connect(self.connect)
        self.wheel_speed = self.get(QLineEdit, "wheelSpeedInput")
        self.wheel_speed.setValidator(QIntValidator(0, 255, self.wheel_speed))
        self.wheel_speed.setText(str(self.motor_speed))
        self.wheel_speed.setPlaceholderText("0-255")
        self.wheel_speed.editingFinished.connect(self.apply_speed)
        self.wheel_speed.returnPressed.connect(self.wheel_speed.clearFocus)
        for name in ("upBtn", "downBtn", "leftBtn", "rightBtn"):
            button = self.get(QPushButton, name)
            button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            button.pressed.connect(lambda n=name: self.set_mouse_drive(n))
            button.released.connect(lambda n=name: self.release_mouse_drive(n))
        self.key_filter = KeyFilter(self)
        app.installEventFilter(self.key_filter)
        self.window.setStyleSheet((base / "theme.qss").read_text(encoding="utf-8"))
        self._init_camera()
        self.update_labels()
        self.update_connection_status()

    def get(self, cls, name):
        widget = self.window.findChild(cls, name)
        if widget is None:
            raise RuntimeError(f"Widget {name} not found in app.ui")
        return widget

    @staticmethod
    def style_state(widget, key, value):
        widget.setProperty(key, value)
        widget.style().unpolish(widget)
        widget.style().polish(widget)
        widget.update()

    def update_labels(self):
        self.arm_label.setText(f"ARM: {round(self.servo_angles[2])}°")
        self.elbow_label.setText(f"ELBOW: {round(self.servo_angles[1])}°")
        self.pivot_label.setText(f"PIVOT: {round(self.servo_angles[0])}°")
        self.grip_label.setText("CLAW: OPEN" if self.grip_open else "CLAW: CLOSED")
        self.grip.setText("OPEN" if self.grip_open else "CLOSE")
        self.grip.setToolTip("Spacebar: toggle claw")
        self.style_state(self.grip, "gripState", "out" if self.grip_open else "in")

    def mouse_control(self, channel, point):
        slider = self.sliders[channel]
        option = QStyleOptionSlider()
        option.initFrom(slider)
        option.orientation = slider.orientation()
        option.minimum = slider.minimum()
        option.maximum = slider.maximum()
        option.sliderPosition = slider.sliderPosition()
        option.sliderValue = slider.value()
        option.singleStep = slider.singleStep()
        option.pageStep = slider.pageStep()
        reversed_axis = (slider.orientation() == Qt.Orientation.Vertical or
                         slider.layoutDirection() == Qt.LayoutDirection.RightToLeft)
        option.upsideDown = slider.invertedAppearance() != reversed_axis
        style = slider.style()
        groove = style.subControlRect(QStyle.ComplexControl.CC_Slider, option,
                                      QStyle.SubControl.SC_SliderGroove, slider)
        handle = style.subControlRect(QStyle.ComplexControl.CC_Slider, option,
                                      QStyle.SubControl.SC_SliderHandle, slider)
        if slider.orientation() == Qt.Orientation.Horizontal:
            start = groove.x()
            span = groove.width() - handle.width()
            position = point.x() - start - handle.width() // 2
        else:
            start = groove.y()
            span = groove.height() - handle.height()
            position = point.y() - start - handle.height() // 2
        value = QStyle.sliderValueFromPosition(slider.minimum(), slider.maximum(),
                    max(0, min(span, position)), max(1, span), option.upsideDown)
        slider.setValue(value)

    def advance_servos(self):
        if not self.connected or self.connection_busy or self.closing:
            return
        # Fixed small increments avoid a large jump after any UI pause.
        step = SERVO_DEGREES_PER_SECOND * SERVO_INTERVAL_MS / 1000.0
        changed = False
        for channel, slider in self.sliders.items():
            offset = slider.value() - CONTROL_CENTER
            if abs(offset) <= CONTROL_DEADZONE:
                continue
            previous = self.servo_angles[channel]
            target = max(0.0, min(180.0, previous + (step if offset > 0 else -step)))
            self.servo_angles[channel] = target
            if round(target) != round(previous):
                self.bluetooth.submit("servo", channel, round(target))
                changed = True
        if changed:
            self.update_labels()

    def toggle_grip(self):
        self.grip_open = not self.grip_open
        self.update_labels()
        if self.connected and not self.connection_busy and not self.closing:
            self.bluetooth.submit("servo", 3, GRIP_OPEN if self.grip_open else GRIP_CLOSED)

    def set_keyboard_drive(self, direction):
        self.keyboard_drive = direction
        self.update_drive()

    def set_mouse_drive(self, direction):
        self.mouse_drive = direction
        self.update_drive()

    def release_mouse_drive(self, direction):
        if self.mouse_drive == direction:
            self.mouse_drive = None
            self.update_drive()

    def update_drive(self):
        direction = self.mouse_drive or self.keyboard_drive
        self.forward.position = NEUTRAL
        self.turn.position = NEUTRAL
        if direction == "upBtn":
            self.forward.position = 98
        elif direction == "downBtn":
            self.forward.position = 0
        elif direction == "rightBtn":
            self.turn.position = 98
        elif direction == "leftBtn":
            self.turn.position = 0
        for name in ("upBtn", "downBtn", "leftBtn", "rightBtn"):
            self.style_state(self.get(QPushButton, name), "active", "true" if name == direction else "false")
        if self.connected and not self.connection_busy and not self.closing:
            self.bluetooth.submit("motor", self.forward.position, self.turn.position)

    def update_connection_status(self, message=None):
        connected = self.connected
        self.status.setText(message if message is not None else (
            f"CONNECTED: {self.connected_port}" if connected else
            (f"DISCONNECTED: {self.last_error}" if self.last_error else "DISCONNECTED — enter port and press ↻")
        ))
        self.status.setToolTip(self.status.text())
        self.style_state(self.status, "connected", "true" if connected else "false")
        self.style_state(self.reconnect_button, "connected", "true" if connected else "false")
        self.reconnect_button.setText("×" if connected else "↻")
        self.reconnect_button.setToolTip("Disconnect" if connected else "Connect / retry")
        enabled = not self.connection_busy and not self.closing
        self.reconnect_button.setEnabled(enabled)
        self.port.setEnabled(enabled)
        self.wheel_speed.setEnabled(enabled)
        for name in ("upBtn", "downBtn", "leftBtn", "rightBtn", "gripBtn",
                     "armSlider", "elbowSlider", "turnSlider"):
            self.get(QWidget, name).setEnabled(enabled)

    @Slot(object)
    def on_hardware_result(self, result):
        self.connected = result["connected"]
        self.connected_port = result["port"]
        self.last_error = result["error"]
        if result["action"] in ("connect", "disconnect"):
            self.connection_busy = False
        if not self.connected:
            self.mouse_drive = None
            self.key_filter.clear()
        if not self.closing and not self.connection_busy:
            self.update_connection_status()

    def connect(self):
        if self.closing or self.connection_busy or self.connected:
            return
        port = self.port.text().strip()
        if not port:
            self.update_connection_status("ENTER A BLUETOOTH COM PORT")
            return
        self.connection_busy = True
        self.mouse_drive = None
        self.key_filter.clear()
        self.update_connection_status("CONNECTING…")
        angles = tuple((c, round(self.servo_angles[c])) for c in (0, 1, 2)) + (
            (3, GRIP_OPEN if self.grip_open else GRIP_CLOSED),)
        self.bluetooth.submit("connect", port, angles, self.motor_speed)

    def toggle_connection(self):
        if self.closing or self.connection_busy:
            return
        if self.connected:
            self.connection_busy = True
            self.mouse_drive = None
            self.key_filter.clear()
            self.update_connection_status("DISCONNECTING…")
            self.bluetooth.submit("disconnect")
        else:
            self.connect()

    def _init_camera(self):
        holder = QStackedLayout(self.feed)
        holder.setContentsMargins(0, 0, 0, 0)
        holder.addWidget(self.feed_placeholder)
        self.feed_placeholder.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.feed_placeholder.setMinimumSize(1, 1)
        self.feed_placeholder.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.placeholder_text = self.feed_placeholder.text()
        self.last_frame = None
        self.camera_status.setText("CAMERA CONNECTING…")
        self.stream = StreamThread(STREAM_URL, self.window)
        self.stream.frame.connect(self._on_frame)
        self.stream.status.connect(self._on_camera_status)
        self.stream.start()
        self.get(QPushButton, "refreshFeedBtn").clicked.connect(self.refresh_camera)

    @Slot(QImage)
    def _on_frame(self, image):
        self.last_frame = image
        self.feed_placeholder.setPixmap(QPixmap.fromImage(image).scaled(
            self.feed_placeholder.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation))

    @Slot(str)
    def _on_camera_status(self, message):
        self.camera_status.setText(message)
        self.camera_status.setToolTip(message)
        if message != "CAMERA CONNECTED":
            self.last_frame = None
            self.feed_placeholder.setPixmap(QPixmap())
            self.feed_placeholder.setWordWrap(True)
            self.feed_placeholder.setText(f"{message}\n{STREAM_URL}")

    def refresh_camera(self):
        if not self.closing:
            self._on_camera_status("CAMERA RECONNECTING…")
            self.stream.reconnect()

    def apply_speed(self):
        try:
            value = int(self.wheel_speed.text().strip())
        except ValueError:
            value = self.motor_speed  # Empty or invalid: keep the old speed.
        value = max(0, min(255, value))
        self.wheel_speed.setText(str(value))
        self.motor_speed = value
        if not self.closing and not self.connection_busy:
            self.bluetooth.submit("speed", value)
        self.update_drive()  # Re-send so a speed change applies while driving.

    def show(self):
        self.window.show()

    def resize_layout(self):
        scale_x = self.window.width() / self.reference_width
        scale_y = self.window.height() / self.reference_height

        for name, (x, y, width, height) in self.reference_geometries.items():
            widget = self.get(QWidget, name)
            widget.setGeometry(
                round(x * scale_x),
                round(y * scale_y),
                max(1, round(width * scale_x)),
                max(1, round(height * scale_y)),
            )

    def shutdown(self):
        if self.closing:
            return
        self.closing = True
        self.servo_timer.stop()
        self.key_filter.clear()
        self.mouse_drive = None
        self.keyboard_drive = None
        self.update_connection_status("CLOSING…")
        self.stream.stop()
        self.bluetooth.submit("disconnect")
        self.close_timer = QTimer(self)
        self.close_timer.timeout.connect(self.finish_shutdown)
        self.close_timer.start(25)

    def finish_shutdown(self):
        if not self.bluetooth.idle() or self.stream.isRunning():
            return
        self.close_timer.stop()
        self.close_ready = True
        self.application.removeEventFilter(self.key_filter)
        self.window.close()

    def wait_for_shutdown(self):
        # Fallback for an external app.quit() after the event loop has stopped.
        if self.close_ready:
            return
        self.shutdown()
        self.bluetooth.pool.waitForDone()
        # The GUI completion slot may be queued when app.quit() stops the loop.
        self.bluetooth.pending.clear()
        self.bluetooth.active = None
        self.bluetooth.submit("disconnect")
        self.bluetooth.pool.waitForDone()
        self.stream.wait()


class StreamThread(QThread):
    """MJPEG reader with bounded buffers, visible errors and nonblocking retry."""
    frame = Signal(QImage)
    status = Signal(str)

    def __init__(self, url, parent=None):
        super().__init__(parent)
        self.url = url
        self.running = True
        self.retry = threading.Event()

    def run(self):
        while self.running:
            self.retry.clear()
            self.status.emit("CAMERA CONNECTING…")
            try:
                request = urllib.request.Request(self.url, headers={"User-Agent": "RobotCamera/1.0"})
                with urllib.request.urlopen(request, timeout=3) as response:
                    buffer = b""
                    connected = False
                    last_frame_time = time.monotonic()
                    last_emit = 0.0
                    while self.running and not self.retry.is_set():
                        chunk = response.read1(16384)
                        if not chunk:
                            raise OSError("Camera closed the stream")
                        buffer += chunk
                        latest = None
                        while True:
                            start = buffer.find(b"\xff\xd8")
                            if start < 0:
                                buffer = buffer[-1:]  # Preserve a split JPEG marker.
                                break
                            if start:
                                buffer = buffer[start:]
                            end = buffer.find(b"\xff\xd9", 2)
                            if end < 0:
                                break
                            jpeg, buffer = buffer[:end + 2], buffer[end + 2:]
                            image = QImage.fromData(jpeg, "JPG")
                            if not image.isNull():
                                latest = image
                                last_frame_time = time.monotonic()
                        if len(buffer) > 4 * 1024 * 1024:
                            raise OSError("Invalid or oversized camera frame")
                        if time.monotonic() - last_frame_time > 5:
                            raise OSError("No JPEG frames received; check the stream URL")
                        if latest is not None:
                            if not connected:
                                connected = True
                                self.status.emit("CAMERA CONNECTED")
                            now = time.monotonic()
                            if now - last_emit >= 1 / 30:
                                self.frame.emit(latest)
                                last_emit = now
            except Exception as exc:
                if self.running and not self.retry.is_set():
                    self.status.emit(f"CAMERA UNAVAILABLE: {exc}")
            if self.running:
                self.retry.wait(2)

    def reconnect(self):
        # Current network read times out within ~3 s; never wait on the GUI.
        self.retry.set()

    def stop(self):
        self.running = False
        self.retry.set()



def main():
    application = QApplication(sys.argv)
    board = MyTelemetrix()  # Offline at startup; user explicitly connects in UI.
    ui = RobotUI(application, board)
    application.aboutToQuit.connect(ui.wait_for_shutdown)
    ui.show()
    return application.exec()


if __name__ == "__main__":
    sys.exit(main())
