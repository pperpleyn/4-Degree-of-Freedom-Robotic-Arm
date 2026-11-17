import importlib
import sys
from pathlib import Path
from typing import TypeVar

from PySide6.QtCore import QEvent, QFile, QObject, QIODevice, QRegularExpression, Qt, QUrl
from PySide6.QtGui import QRegularExpressionValidator
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWidgets import (
    QApplication,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QStackedLayout,
    QSlider,
    QWidget,
)

STREAM_URL = "http://192.168.254.123:81/stream"
BAUD_RATE = 9600
NEUTRAL = 50
DEADZONE = 2
WidgetType = TypeVar("WidgetType", bound=QWidget)
PORT_NAME = "COM4"

class KeyboardController(QObject):
    key_commands = {
        Qt.Key.Key_W: "F",
        Qt.Key.Key_S: "B",
        Qt.Key.Key_A: "L",
        Qt.Key.Key_D: "R",
    }

    def __init__(self, controller):
        super().__init__(controller.window)
        self.controller = controller
        self.pressed_keys = []

    def eventFilter(self, watched, event):
        window = self.controller.window
        if not isinstance(watched, QWidget) or (
            watched is not window and not window.isAncestorOf(watched)
        ):
            return False

        if watched is window and event.type() == QEvent.Type.Resize:
            self.controller.resize_layout()
            return False

        if event.type() == QEvent.Type.WindowDeactivate:
            self.reset()
            return False

        if isinstance(window.focusWidget(), QLineEdit):
            return False

        if event.type() == QEvent.Type.KeyPress:
            key = event.key()
            if key not in self.key_commands:
                return False
            if not event.isAutoRepeat():
                if key in self.pressed_keys:
                    self.pressed_keys.remove(key)
                self.pressed_keys.append(key)
                self.apply_keys()
            return True

        if event.type() == QEvent.Type.KeyRelease:
            key = event.key()
            if key not in self.key_commands:
                return False
            if not event.isAutoRepeat() and key in self.pressed_keys:
                self.pressed_keys.remove(key)
                self.apply_keys()
            return True

        return False

    def apply_keys(self):
        self.controller.keyboard_direction = (
            self.key_commands[self.pressed_keys[-1]] if self.pressed_keys else None
        )
        self.controller.update_drive()

    def reset(self):
        if not self.pressed_keys:
            return
        self.pressed_keys.clear()
        self.apply_keys()


class RobotController:
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

    def __init__(self, application):
        self.application = application
        self.serial_connection = None
        self.gripper_open = None
        self.keyboard_direction = None
        self.button_direction = None

        script_dir = Path(__file__).resolve().parent
        ui_file = QFile(str(script_dir / "WebApp" / "app.ui"))
        if not ui_file.open(QIODevice.OpenModeFlag.ReadOnly):
            raise RuntimeError(f"Unable to open the UI file: {ui_file.fileName()}")
        try:
            self.window = QUiLoader().load(ui_file)
        finally:
            ui_file.close()
        if self.window is None:
            raise RuntimeError("Qt could not load app.ui.")

        self.arm_slider = self.require_widget(QSlider, "armSlider")
        self.elbow_slider = self.require_widget(QSlider, "elbowSlider")
        self.turn_slider = self.require_widget(QSlider, "turnSlider")
        self.port_edit = self.require_widget(QLineEdit, "portNameEdit")
        self.wheel_speed_input = self.require_widget(QLineEdit, "wheelSpeedInput")
        self.wheel_speed_input.setMaxLength(3)
        self.wheel_speed_input.setValidator(
            QRegularExpressionValidator(
                QRegularExpression(r"(?:[0-9]{0,2}|100)"), self.wheel_speed_input
            )
        )
        self.connection_status = self.require_widget(QLabel, "connectionStatus")
        self.turn_status = self.require_widget(QLabel, "turnStatus")
        self.arm_status = self.require_widget(QLabel, "armValue")
        self.elbow_status = self.require_widget(QLabel, "elbowValue")
        self.grip_status = self.require_widget(QLabel, "gripStatus")
        self.grip_button = self.require_widget(QPushButton, "gripBtn")
        self.grip_button.setText("Close")

        feed = self.require_widget(QWidget, "Feed")
        self.feed_placeholder = self.require_widget(QLabel, "feedPlaceholder")
        feed_layout = QStackedLayout(feed)
        feed_layout.setContentsMargins(0, 0, 0, 0)
        feed_layout.addWidget(self.feed_placeholder)
        self.web_view = QWebEngineView(feed)
        self.web_view.setUrl(QUrl(STREAM_URL))
        feed_layout.addWidget(self.web_view)
        feed_layout.setCurrentWidget(self.feed_placeholder)
        self.feed_layout = feed_layout
        self.web_view.loadFinished.connect(self.on_camera_load_finished)
        self.require_widget(QPushButton, "refreshFeedBtn").clicked.connect(
            self.web_view.reload
        )

        self.arm_slider.valueChanged.connect(self.update_arm)
        self.elbow_slider.valueChanged.connect(self.update_elbow)
        self.turn_slider.valueChanged.connect(self.update_drive)
        self.turn_slider.sliderReleased.connect(
            lambda: self.turn_slider.setValue(NEUTRAL)
        )
        for name, command in (
            ("upBtn", "F"),
            ("downBtn", "B"),
            ("leftBtn", "L"),
            ("rightBtn", "R"),
        ):
            button = self.require_widget(QPushButton, name)
            button.pressed.connect(
                lambda command=command: self.set_button_direction(command)
            )
            button.released.connect(
                lambda command=command: self.release_button_direction(command)
            )
        self.require_widget(QPushButton, "reconnBtn").clicked.connect(
            self.toggle_connection
        )
        self.grip_button.clicked.connect(self.toggle_gripper)

        self.keyboard = KeyboardController(self)
        application.installEventFilter(self.keyboard)
        self.window.installEventFilter(self.keyboard)
        self.window.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.resize_layout()
        self.update_drive()
        self.update_arm()
        self.update_elbow()
        self.window.setStyleSheet(self.stylesheet())

    def require_widget(self, widget_type: type[WidgetType], name: str) -> WidgetType:
        widget = self.window.findChild(widget_type, name)
        if widget is None:
            raise RuntimeError(f"Required widget {name!r} is missing from app.ui.")
        return widget

    @staticmethod
    def stylesheet():
        return """
            QWidget {
                background-color: #eff5ef;
                color: #203b29;
                font-family: Arial;
                font-size: 12px;
            }
            QWidget#Feed {
                background-color: #dcebe0;
                border: 2px solid #3e7950;
                border-radius: 12px;
            }
            QLabel#feedPlaceholder {
                background-color: #dcebe0;
                color: #36734a;
                font-size: 15px;
                font-weight: 700;
            }
            QLabel#feedTitle {
                background: transparent;
                color: #245f37;
                font-size: 17px;
                font-weight: 700;
            }
            QLabel#turnTitle, QLabel#armTitle, QLabel#elbowTitle {
                background-color: #e0eee2;
                border: 1px solid #bed8c2;
                border-radius: 6px;
                color: #245f37;
                font-size: 14px;
                font-weight: 700;
                padding: 3px;
            }
            QLabel#turnStatus, QLabel#armValue, QLabel#elbowValue {
                background-color: #e5f1e7;
                border: 1px solid #c6ddca;
                border-radius: 8px;
                color: #237b40;
                font-size: 11px;
                font-weight: 700;
                padding-left: 8px;
            }
            QLabel#connectionStatus {
                background-color: #fff3d8;
                border: 1px solid #e6ca82;
                border-radius: 8px;
                color: #8a6413;
                font-size: 10px;
                font-weight: 700;
            }
            QLabel#connectionStatus[connected="true"] {
                background-color: #d6efdc;
                border-color: #75b985;
                color: #1f7138;
            }
            QLabel#cameraStatus, QLabel#gripStatus {
                background: transparent;
            }
            QLineEdit {
                background-color: #fbfdfb;
                border: 1px solid #7eaa86;
                border-radius: 8px;
                color: #203b29;
                padding: 5px 9px;
                selection-background-color: #a7d6b1;
            }
            QLineEdit:focus {
                border: 2px solid #329456;
            }
            QLabel#bluetoothLabel, QLabel#comPortLabel {
                background: transparent;
                color: #245f37;
                font-size: 13px;
                font-weight: 700;
            }
            QPushButton {
                background-color: #d3e9d7;
                border: 1px solid #438052;
                border-radius: 8px;
                color: #205a32;
                font-weight: 700;
                padding: 4px 8px;
            }
            QPushButton:hover {
                background-color: #c0e0c8;
                border-color: #267744;
            }
            QPushButton:pressed {
                background-color: #a9d2b3;
            }
            QPushButton#reconnBtn {
                background-color: #cbe7d1;
                border: 1px solid #398050;
                border-radius: 10px;
                color: #205a32;
                font-family: "Segoe UI Symbol";
                font-size: 24px;
                padding: 0;
            }
            QPushButton#reconnBtn[connected="true"] {
                background-color: #318b4d;
                border-color: #236f3b;
                color: #ffffff;
            }
            QPushButton#upBtn, QPushButton#downBtn, QPushButton#leftBtn,
            QPushButton#rightBtn {
                background-color: #f7fbf7;
                border: 2px solid #75a982;
                border-radius: 28px;
                color: #267744;
                font-family: "Segoe UI Symbol";
                font-size: 22px;
                font-weight: 400;
                padding: 0;
            }
            QPushButton#upBtn:hover, QPushButton#downBtn:hover,
            QPushButton#leftBtn:hover, QPushButton#rightBtn:hover {
                background-color: #dcefe0;
                border-color: #398050;
            }
            QPushButton#gripBtn {
                background-color: #e8ece8;
                border-color: #87968a;
                color: #46564a;
            }
            QPushButton#gripBtn[gripState="in"] {
                background-color: #cbe7d1;
                border-color: #398050;
                color: #205a32;
            }
            QPushButton#gripBtn[gripState="in"]:hover {
                background-color: #b9ddc0;
                border-color: #267744;
            }
            QPushButton#gripBtn[gripState="out"] {
                background-color: #f4e0df;
                border-color: #b45b55;
                color: #8a302b;
            }
            QPushButton#gripBtn[gripState="out"]:hover {
                background-color: #edcecb;
                border-color: #99453f;
            }
            QPushButton#gripBtn {
                border-radius: 51px;
                font-size: 16px;
            }
            QSlider::groove:vertical {
                background-color: #d5e8d8;
                border: 1px solid #5b9669;
                border-radius: 8px;
                width: 18px;
            }
            QSlider::handle:vertical {
                background-color: #319453;
                border: 2px solid #f8fff9;
                border-radius: 8px;
                height: 20px;
                margin: 0 -9px;
            }
            QSlider::groove:horizontal {
                background-color: #d5e8d8;
                border: 1px solid #5b9669;
                border-radius: 8px;
                height: 16px;
            }
            QSlider::handle:horizontal {
                background-color: #319453;
                border: 2px solid #f8fff9;
                border-radius: 8px;
                width: 20px;
                margin: -10px 0;
            }
        """

    def toggle_connection(self):
        if self.serial_connection is not None:
            self.disconnect_robot()
            return

        port = self.port_edit.text().strip()
        if not port:
            self.set_connection_status("ENTER A COM PORT", connected=False)
            return

        try:
            serial_module = importlib.import_module("serial")
            connection = serial_module.Serial(
                port=port,
                baudrate=BAUD_RATE,
                timeout=0.1,
                write_timeout=0.25,
            )
        except (ImportError, OSError, ValueError) as error:
            self.set_connection_status(f"CONNECTION FAILED: {error}", connected=False)
            QMessageBox.warning(
                self.window,
                "Robot connection failed",
                f"Could not connect to {port}.\n\n{error}",
            )
            return

        self.serial_connection = connection
        reconnect_button = self.require_widget(QPushButton, "reconnBtn")
        reconnect_button.setText("×")
        reconnect_button.setProperty("connected", True)
        reconnect_button.style().unpolish(reconnect_button)
        reconnect_button.style().polish(reconnect_button)
        self.set_connection_status(f"CONNECTED · {port}", connected=True)

    def disconnect_robot(self):
        connection = self.serial_connection
        self.serial_connection = None
        reconnect_button = self.require_widget(QPushButton, "reconnBtn")
        reconnect_button.setText("✓")
        reconnect_button.setProperty("connected", False)
        reconnect_button.style().unpolish(reconnect_button)
        reconnect_button.style().polish(reconnect_button)
        if connection is not None:
            try:
                connection.close()
            except OSError as error:
                self.set_connection_status(f"DISCONNECT ERROR: {error}", connected=False)
                QMessageBox.warning(
                    self.window,
                    "Robot disconnect failed",
                    str(error),
                )
                return
        self.set_connection_status("DISCONNECTED", connected=False)

    def send_command(self, command: str):
        if self.serial_connection is None:
            return
        try:
            self.serial_connection.write(f"{command}\n".encode("ascii"))
        except OSError as error:
            self.disconnect_robot()
            self.set_connection_status(f"COMMAND FAILED: {error}", connected=False)
            QMessageBox.warning(
                self.window,
                "Robot command failed",
                f"The connection was closed after a write error.\n\n{error}",
            )

    def update_drive(self):
        turn = self.turn_slider.value() - NEUTRAL
        command = self.button_direction or self.keyboard_direction
        if command is None:
            command = (
                "R" if turn > DEADZONE else "L" if turn < -DEADZONE else "S"
            )
        self.turn_status.setText(f"L/R: {self.turn_slider.value()}")
        self.send_command(command)

    def update_arm(self):
        value = self.arm_slider.value()
        self.arm_status.setText(f"ARM: {value}")
        self.send_command(f"A:{value}")

    def update_elbow(self):
        value = self.elbow_slider.value()
        self.elbow_status.setText(f"ELBOW: {value}")
        self.send_command(f"E:{value}")

    def set_button_direction(self, command: str):
        self.button_direction = command
        self.update_drive()

    def release_button_direction(self, command: str):
        if self.button_direction == command:
            self.button_direction = None
            self.update_drive()

    def resize_layout(self):
        scale_x = self.window.width() / self.reference_width
        scale_y = self.window.height() / self.reference_height
        for name, (x, y, width, height) in self.reference_geometries.items():
            widget = self.require_widget(QWidget, name)
            widget.setGeometry(
                round(x * scale_x),
                round(y * scale_y),
                max(1, round(width * scale_x)),
                max(1, round(height * scale_y)),
            )

    def toggle_gripper(self):
        self.gripper_open = not self.gripper_open
        self.send_command("O" if self.gripper_open else "P")
        state = "OPEN" if self.gripper_open else "CLOSED"
        self.grip_status.setText(f"Gripper: {state}")
        self.grip_button.setText("Open" if self.gripper_open else "Close")
        self.grip_button.setProperty(
            "gripState", "out" if self.gripper_open else "in"
        )
        self.grip_button.style().unpolish(self.grip_button)
        self.grip_button.style().polish(self.grip_button)
        self.grip_button.update()

    def on_camera_load_finished(self, succeeded: bool):
        status = "CAMERA CONNECTED" if succeeded else "CAMERA UNAVAILABLE"
        self.require_widget(QLabel, "cameraStatus").setText(status)
        current = self.web_view if succeeded else self.feed_placeholder
        self.feed_layout.setCurrentWidget(current)

    def set_connection_status(self, message: str, connected: bool):
        self.connection_status.setText(message)
        self.connection_status.setProperty("connected", connected)
        self.connection_status.style().unpolish(self.connection_status)
        self.connection_status.style().polish(self.connection_status)

    def show(self):
        self.window.show()

    def shutdown(self):
        self.send_command("S")
        self.application.removeEventFilter(self.keyboard)
        self.window.removeEventFilter(self.keyboard)
        self.disconnect_robot()


def main():
    application = QApplication(sys.argv)
    controller = RobotController(application)
    application.aboutToQuit.connect(controller.shutdown)
    controller.show()
    return application.exec()


if __name__ == "__main__":
    sys.exit(main())