import sys
from pathlib import Path
from utils.myTelemetrix import MyTelemetrix
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QApplication, QVBoxLayout
from PySide6.QtCore import QFile, QIODevice, QUrl
from PySide6.QtWebEngineWidgets import QWebEngineView

STREAM_URL = "http://192.168.254.123:81/stream"
PORT_NAME = "COM4"
# EDITOR'S NOTE: '/dev/rfcomm0' is a virtual port for Arjay's Nobara OS (Linux Fedora Distro)
# For anyone testing this, please change this variable!

class MyApp:
    def __init__(self, telemetrixObject):
        script_dir = Path(__file__).resolve().parent
        ui_path = script_dir / "app.ui"

        ui_file = QFile(str(ui_path))
        ui_file.open(QIODevice.ReadOnly)
        loader = QUiLoader()
        self.window = loader.load(ui_file)
        ui_file.close()

        self.board = telemetrixObject

        # INIT Widget Functions
        # Webview
        self.web_view = QWebEngineView()
        self.web_view.load(QUrl(STREAM_URL))

        layout = QVBoxLayout(self.window.Feed)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.web_view)

        # Ensure sliders start at default 49 if not set in .ui
        self.window.forwardSlider.setValue(49)
        self.window.turnSlider.setValue(49)

        # SLIDER SIGNALS & UI LOGIC
        # LEFT or RIGHT Slider bindings
        self.window.turnSlider.valueChanged.connect(self.on_turn_changed)
        self.window.turnSlider.sliderReleased.connect(self.reset_to_neutralT)

        # FORWARD or BACKWARD Slider bindings
        self.window.forwardSlider.valueChanged.connect(self.on_forward_changed)
        self.window.forwardSlider.sliderReleased.connect(self.reset_to_neutralF)

        # Reconnect Panel
        self.window.reconnBtn.clicked.connect(lambda: self.board.reconn(self.window.portNameEdit.text()))

        # Initial stop state
        self.board.update_motors(self.window.forwardSlider, self.window.turnSlider)

    def show(self):
        self.window.show()

    # UI LOGIC & HARDWARE TRIGGER
    def on_turn_changed(self, value):
        offset = value - 49
        self.window.turnStatus.setText(f"Slider Value: {value} | Command Offset: {offset:+d}")
        # Send current states of both sliders to telemetrix
        self.board.update_motors(self.window.forwardSlider, self.window.turnSlider)

    def reset_to_neutralT(self):
        self.window.turnSlider.setValue(49)

    def on_forward_changed(self, value):
        offset = value - 49
        self.window.forwardStatus.setText(f"Slider Value: {value} | Command Offset: {offset:+d}")
        # Send current states of both sliders to telemetrix
        self.board.update_motors(self.window.forwardSlider, self.window.turnSlider)

    def reset_to_neutralF(self):
        self.window.forwardSlider.setValue(49)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    conn_tele = MyTelemetrix(com_port=PORT_NAME)
    tele_app = MyApp(telemetrixObject = conn_tele)
    tele_app.show()
    app.aboutToQuit.connect(conn_tele.close)
    sys.exit(app.exec())