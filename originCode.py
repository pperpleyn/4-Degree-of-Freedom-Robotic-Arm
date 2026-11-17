import sys
from pathlib import Path
from utils.myTelemetrix import MyTelemetrix
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QApplication, QVBoxLayout
from PySide6.QtCore import QFile, QIODevice, QUrl
from PySide6.QtWebEngineWidgets import QWebEngineView

STREAM_URL = "http://192.168.4.1:81/stream"
PORT_NAME = "COM5"

# Define PCA9685 Servo Channels
SERVO0 = 0
SERVO1 = 1
SERVO2 = 2
SERVO3 = 3 

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

        # Ensure motor sliders start at default 49
        self.window.forwardSlider.setValue(49)
        self.window.turnSlider.setValue(49)

        # --- INITIALIZE ALL 4 SERVO SLIDERS ---
        # Format: (slider_name, channel_index, min_range, max_range, default_value)
        servo_configs = [
            ('servo0Slider', SERVO0, 0, 180, 90),
            ('servo1Slider', SERVO1, 0, 180, 90),
            ('servo2Slider', SERVO2, 0, 180, 90),
            ('servo3Slider', SERVO3, 25, 100, 90),  # Preserved your custom range for servo 3
        ]

        for slider_name, channel, min_val, max_val, default_val in servo_configs:
            if hasattr(self.window, slider_name):
                slider = getattr(self.window, slider_name)
                slider.setRange(min_val, max_val)
                slider.setValue(default_val)
                
                # Bind valueChanged signal using a default argument (ch=channel) to prevent closure bugs
                slider.valueChanged.connect(lambda value, ch=channel: self.on_servo_changed(ch, value))
                
                # Send initial position to hardware board
                if self.board:
                    self.board.set_servo_angle(channel, default_val)

        # SLIDER SIGNALS & UI LOGIC
        # LEFT or RIGHT Slider bindings
        self.window.turnSlider.valueChanged.connect(self.on_turn_changed)
        self.window.turnSlider.sliderReleased.connect(self.reset_to_neutralT)

        # FORWARD or BACKWARD Slider bindings
        self.window.forwardSlider.valueChanged.connect(self.on_forward_changed)
        self.window.forwardSlider.sliderReleased.connect(self.reset_to_neutralF)

        if self.board:
            # Reconnect Panel
            self.window.reconnBtn.clicked.connect(lambda: self.board.reconn(self.window.portNameEdit.text()))
            # Initial stop state
            self.board.update_motors(self.window.forwardSlider, self.window.turnSlider)

    def show(self):
        self.window.show()

    # --- DYNAMIC SERVO UI LOGIC (Handles all 4 servos) ---
    def on_servo_changed(self, channel, value):
        # Dynamically update the matching status label if it exists in your .ui (e.g. servo0Status, servo1Status...)
        status_label_name = f"servo{channel}Status"
        if hasattr(self.window, status_label_name):
            getattr(self.window, status_label_name).setText(f"Angle: {value}°")
        
        # Send the updated value to the correct servo channel via Telemetrix
        if self.board:
            self.board.set_servo_angle(channel, value)

    # UI LOGIC & HARDWARE TRIGGER
    def on_turn_changed(self, value):
        offset = value - 49
        self.window.turnStatus.setText(f"Slider Value: {value} | Command Offset: {offset:+d}")
        self.board.update_motors(self.window.forwardSlider, self.window.turnSlider)

    def reset_to_neutralT(self):
        self.window.turnSlider.setValue(49)

    def on_forward_changed(self, value):
        offset = value - 49
        self.window.forwardStatus.setText(f"Slider Value: {value} | Command Offset: {offset:+d}")
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