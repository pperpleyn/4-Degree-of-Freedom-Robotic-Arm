"""Robotic arm controller using the provided app.ui and MyTelemetrix.

Keys: WASD wheels; I/K arm (servo 2); U/J elbow (servo 1);
N/M pivot (servo 0); Space claw toggle (servo 3).
"""
import sys
from pathlib import Path
import urllib.request

from PySide6.QtCore import QEvent, QFile, QIODevice, QObject, Qt, QUrl, Signal, Slot, QThread, QTimer, QRunnable, QThreadPool, QRegularExpression
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QApplication, QLabel, QLineEdit, QPushButton, QSlider, QStackedLayout, QWidget, QSizePolicy
from PySide6.QtGui import QImage, QPixmap, QIntValidator, QRegularExpressionValidator
from zope import event

try:
    from PySide6.QtWebEngineWidgets import QWebEngineView
except ImportError:
    QWebEngineView = None

from utils.myTelemetrix import MyTelemetrix

PORT_NAME = "COM5"   # Change to your Bluetooth COM port; no connection is attempted at startup.
STREAM_URL = "http://10.49.174.193:81/stream"
SERVO_STEP = 5
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

    def eventFilter(self, watched, event):
        if watched is self.owner.window and event.type() == QEvent.Type.Close:
            if not self.owner.close_ready:
                event.ignore()
                self.owner.shutdown()
                return True
        if watched is self.owner.window and event.type() == QEvent.Type.Resize:
            self.owner.resize_layout()
            return False
        if self.owner.closing or self.owner.connection_busy:
            return False
        if not isinstance(watched, QWidget) or (watched is not self.owner.window and
                not self.owner.window.isAncestorOf(watched)):
            return False
        kind = event.type()
        if kind == QEvent.Type.WindowDeactivate:
            self.clear()
            return False
        if kind not in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            return False
        if kind == QEvent.Type.KeyRelease and event.key() in self.servo_keys:
            if not event.isAutoRepeat():
                channel, _ = self.servo_keys[event.key()]

                slider = {
                    2: self.owner.arm,
                    1: self.owner.elbow,
                    0: self.owner.pivot,
                }[channel]

                slider.setValue(90)

            return True
        if isinstance(self.owner.window.focusWidget(), QLineEdit):
            return False  # Permit normal typing in the COM port entry.
        key = event.key()
        pressed = kind == QEvent.Type.KeyPress
        if key in self.drive_keys:
            if event.isAutoRepeat():
                return True
            if pressed:
                if key not in self.held:
                    self.held.append(key)
            elif key in self.held:
                self.held.remove(key)
            self.owner.set_keyboard_drive(self.drive_keys[self.held[-1]] if self.held else None)
            return True
        if key in self.servo_keys:
            channel, sign = self.servo_keys[key]
            if pressed:
                self.owner.step_servo(channel, sign * SERVO_STEP)
            return True 
            
        if key == Qt.Key.Key_Space:
            if pressed and not event.isAutoRepeat():
                self.owner.toggle_grip()
            return True

        elif not event.isAutoRepeat():
            slider = {
                2: self.owner.arm,
                1: self.owner.elbow,
                0: self.owner.pivot,
            }[channel]

            slider.setValue(90)

            return True
        return False

    def clear(self):
        self.held.clear()
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
        "cameraStatus": (0, 0, 1, 1),
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
        self.turnSlider = self.get(QSlider, "turnSlider")
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
        self.arm.valueChanged.connect(lambda v: self.on_servo(2, v))
        self.elbow.valueChanged.connect(lambda v: self.on_servo(1, v))
        self.pivot.valueChanged.connect(lambda v: self.on_servo(0, v))
        for slider in (self.arm, self.elbow, self.pivot):
            slider.sliderReleased.connect(
                lambda s=slider: s.setValue(90)
            )   
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
        self.arm_label.setText(f"ARM: {self.arm.value()}°")
        self.elbow_label.setText(f"ELBOW: {self.elbow.value()}°")
        self.pivot_label.setText(f"PIVOT: {self.pivot.value()}°")
        self.grip_label.setText("CLAW: OPEN" if self.grip_open else "CLAW: CLOSED")
        self.grip.setText("OPEN" if self.grip_open else "CLOSE")
        self.grip.setToolTip("Spacebar: toggle claw")
        self.style_state(self.grip, "gripState", "out" if self.grip_open else "in")

    def step_servo(self, channel, amount):
        slider = {2: self.arm, 1: self.elbow, 0: self.pivot}[channel]
        slider.setValue(max(slider.minimum(), min(slider.maximum(), slider.value() + amount)))

    def on_servo(self, channel, value):
        self.update_labels()
        if self.connected and not self.connection_busy and not self.closing:
            self.bluetooth.submit("servo", channel, value)

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
        angles = ((0, self.pivot.value()), (1, self.elbow.value()),
                  (2, self.arm.value()), (3, GRIP_OPEN if self.grip_open else GRIP_CLOSED))
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

    @Slot(QImage)
    def _on_frame(self, image):
        self.last_frame = image
        self.feed_placeholder.setPixmap(QPixmap.fromImage(image).scaled(
            self.feed_placeholder.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation))

    @Slot(bool)
    def _on_camera_status(self, connected):
        self.camera_status.setText("CAMERA CONNECTED" if connected else "CAMERA UNAVAILABLE")
        if not connected:
            self.last_frame = None
            self.feed_placeholder.setPixmap(QPixmap())
            self.feed_placeholder.setText(self.placeholder_text)

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
    """Reads an MJPEG stream and emits each JPEG frame as a QImage."""
    frame = Signal(QImage)
    status = Signal(bool)

    def __init__(self, url, parent=None):
        super().__init__(parent)
        self.url = url
        self.running = True

    def run(self):
        connected = False
        while self.running:
            try:
                with urllib.request.urlopen(self.url, timeout=3) as response:
                    buffer = b""
                    while self.running:
                        chunk = response.read(4096)
                        if not chunk:
                            break
                        buffer += chunk
                        start = buffer.find(b"\xff\xd8")
                        end = buffer.find(b"\xff\xd9", start + 2) if start != -1 else -1
                        if start != -1 and end != -1:
                            jpeg = buffer[start:end + 2]
                            buffer = buffer[end + 2:]
                            image = QImage.fromData(jpeg, "JPG")
                            if not image.isNull():
                                if not connected:
                                    connected = True
                                    self.status.emit(True)
                                self.frame.emit(image)
                        elif start == -1 and len(buffer) > 65536:
                            buffer = b""  # Discard junk so the buffer can't grow forever.
            except Exception:
                pass
            if connected:
                connected = False
                self.status.emit(False)
            for _ in range(20):  # Wait ~2 s before retrying, but stay stoppable.
                if not self.running:
                    return
                self.msleep(100)

    def stop(self):
        self.running = False



def main():
    application = QApplication(sys.argv)
    board = MyTelemetrix()  # Offline at startup; user explicitly connects in UI.
    ui = RobotUI(application, board)
    application.aboutToQuit.connect(ui.wait_for_shutdown)
    ui.show()
    return application.exec()


if __name__ == "__main__":
    sys.exit(main())
