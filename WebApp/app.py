"""Robotic arm controller using the provided app.ui and MyTelemetrix.

Keys: WASD wheels; I/K arm (servo 2); U/J elbow (servo 1);
N/M pivot (servo 0); Space claw toggle (servo 3).
"""
import sys
from pathlib import Path
import urllib.request

from PySide6.QtCore import QEvent, QFile, QIODevice, QObject, Qt, QUrl, Signal, QThread, QTimer
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QApplication, QLabel, QLineEdit, QPushButton, QSlider, QStackedLayout, QWidget, QSizePolicy
from PySide6.QtGui import QImage, QPixmap, QIntValidator

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
        if not isinstance(watched, QWidget) or (watched is not self.owner.window and
                not self.owner.window.isAncestorOf(watched)):
            return False
        kind = event.type()
        if kind == QEvent.Type.WindowDeactivate:
            self.clear()
            return False
        if kind not in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            return False
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
            if pressed:
                channel, sign = self.servo_keys[key]
                self.owner.step_servo(channel, sign * SERVO_STEP)
            return True
        if key == Qt.Key.Key_Space:
            if pressed and not event.isAutoRepeat():
                self.owner.toggle_grip()
            return True
        return False

    def clear(self):
        self.held.clear()
        self.owner.set_keyboard_drive(None)


class RobotUI:
    def __init__(self, app, board):
        self.application = app
        self.board = board  # Exactly one instance of MyTelemetrix.
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
        self.arm.valueChanged.connect(lambda v: self.on_servo(2, v))
        self.elbow.valueChanged.connect(lambda v: self.on_servo(1, v))
        self.pivot.valueChanged.connect(lambda v: self.on_servo(0, v))
        self.grip.clicked.connect(self.toggle_grip)
        self.reconnect_button.clicked.connect(self.toggle_connection)
        self.port.returnPressed.connect(self.connect)
        self.wheel_speed = self.get(QLineEdit, "wheelSpeedInput")
        self.wheel_speed.setValidator(QIntValidator(0, 255, self.wheel_speed))
        self.wheel_speed.setText(str(self.board.motor_speed))
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
        self.window.installEventFilter(self.key_filter)
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
        if self.board.connected:
            try:
                self.board.set_servo_angle(channel, value)
            except Exception as error:
                self.hardware_error(error)

    def toggle_grip(self):
        self.grip_open = not self.grip_open
        self.update_labels()
        if self.board.connected:
            try:
                self.board.set_servo_angle(3, GRIP_OPEN if self.grip_open else GRIP_CLOSED)
            except Exception as error:
                self.hardware_error(error)

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
        if self.board.connected:
            try:
                self.board.update_motors(self.forward, self.turn)
            except Exception as error:
                self.hardware_error(error)

    def update_connection_status(self, message=None):
        connected = self.board.connected
        self.status.setText(message if message is not None else (
            f"CONNECTED: {self.board.com_port}" if connected else
            (f"DISCONNECTED: {self.board.last_error}" if self.board.last_error else "DISCONNECTED — enter port and press ↻")
        ))
        self.status.setToolTip(self.status.text())
        self.style_state(self.status, "connected", "true" if connected else "false")
        self.style_state(self.reconnect_button, "connected", "true" if connected else "false")
        self.reconnect_button.setText("×" if connected else "↻")
        self.reconnect_button.setToolTip("Disconnect" if connected else "Connect / retry")

    def connect(self):
        port = self.port.text().strip()
        if not port:
            self.update_connection_status("ENTER A BLUETOOTH COM PORT")
            return
        self.key_filter.clear()
        self.mouse_drive = None
        self.update_connection_status("CONNECTING…")
        QApplication.processEvents()
        if not self.board.reconn(port):
            self.update_connection_status()
            return
        try:
            # Restore all four servos when the board is reconnected.
            for channel, angle in ((0, self.pivot.value()), (1, self.elbow.value()),
                                   (2, self.arm.value()), (3, GRIP_OPEN if self.grip_open else GRIP_CLOSED)):
                self.board.set_servo_angle(channel, angle)
            self.board.stop_motors()
        except Exception as error:
            self.hardware_error(error)
            return
        self.update_connection_status()

    def toggle_connection(self):
        if self.board.connected:
            self.key_filter.clear()
            self.mouse_drive = None
            self.board.close()
            self.update_connection_status()
            self.update_drive()
        else:
            self.connect()

    def hardware_error(self, error):
        self.board.last_error = str(error)
        self.board.close()
        self.update_connection_status()

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

    def _on_frame(self, image):
        self.last_frame = image
        self.feed_placeholder.setPixmap(QPixmap.fromImage(image).scaled(
            self.feed_placeholder.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation))

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
            value = self.board.motor_speed  # Empty or invalid: keep the old speed.
        value = max(0, min(255, value))
        self.wheel_speed.setText(str(value))
        self.board.set_motor_speed(value)
        self.update_drive()  # Re-send so a speed change applies while driving.

    def show(self):
        self.window.show()

    def shutdown(self):
        if self.closing:
            return
        self.closing = True
        self.application.removeEventFilter(self.key_filter)
        self.window.removeEventFilter(self.key_filter)
        self.stream.stop()
        self.board.close()

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
                with urllib.request.urlopen(self.url, timeout=15) as response:
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
        self.wait(6000)



def main():
    application = QApplication(sys.argv)
    board = MyTelemetrix()  # Offline at startup; user explicitly connects in UI.
    ui = RobotUI(application, board)
    application.aboutToQuit.connect(ui.shutdown)
    ui.show()
    return application.exec()


if __name__ == "__main__":
    sys.exit(main())
