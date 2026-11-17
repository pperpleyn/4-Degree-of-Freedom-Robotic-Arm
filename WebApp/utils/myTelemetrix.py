"""Telemetrix motor driver and PCA9685 servo control for the robot.

Uses the same motor pin mapping, steering priority, servo pulse widths, and
public methods as originCode.py's original MyTelemetrix.
"""
import time
from telemetrix import telemetrix

IN1, IN2, IN3, IN4 = 2, 3, 7, 8
EN1, EN2 = 5, 6
PCA9685_ADDR = 0x40
NEUTRAL = 49
DEADZONE = 2
MOTOR_SPEED = 140


class MyTelemetrix:
    def __init__(self, com_port=None):
        self.board = None
        self.com_port = None
        self.last_error = ""
        self.servo_min_us = 500
        self.servo_max_us = 2400
        self.motor_speed = MOTOR_SPEED
        if com_port is not None:
            self.reconn(com_port)

    @property
    def connected(self):
        return self.board is not None

    def _initialize_board(self):
        """Run on EVERY connection, including after a reconnect."""
        for pin in (IN1, IN2, IN3, IN4):
            self.board.set_pin_mode_digital_output(pin)
        self.board.set_pin_mode_analog_output(EN1)
        self.board.set_pin_mode_analog_output(EN2)
        self.board.set_pin_mode_i2c()
        time.sleep(0.1)
        # PCA9685: sleep -> 50 Hz prescale -> wake + auto-increment.
        self.board.i2c_write(PCA9685_ADDR, [0x00, 0x10])
        time.sleep(0.01)
        self.board.i2c_write(PCA9685_ADDR, [0xFE, 121])
        time.sleep(0.01)
        self.board.i2c_write(PCA9685_ADDR, [0x00, 0x20])
        time.sleep(0.01)
        # Must start with the drive outputs OFF. Servo positions get restored
        # by the controller after connection succeeds.
        self.stop_motors()
        self.set_servo_angle(3, 90)

    def reconn(self, com_port=None):
        """Try the specified port, leaving board=None on failure.

        Never auto-detect: an empty Bluetooth port is a disconnected state.
        """
        port = str(com_port).strip() if com_port is not None else (self.com_port or "")
        self.close()
        self.last_error = ""
        if not port:
            self.com_port = None
            self.last_error = "Enter a Bluetooth COM port, then press reconnect."
            return False
        candidate = None
        try:
            candidate = telemetrix.Telemetrix(com_port=port, arduino_wait=10, sleep_tune=0.05)
            self.board = candidate
            self._initialize_board()
            self.com_port = port
            return True
        except Exception as error:
            self.last_error = str(error)
            print(f"Telemetrix reconnect failed: {error}")
            if self.board is not None:
                self.close()
            elif candidate is not None:
                try:
                    candidate.shutdown()
                except Exception:
                    pass
            return False

    def close(self):
        board = self.board
        self.board = None
        if board is None:
            return
        try:
            for pin in (IN1, IN2, IN3, IN4):
                board.digital_write(pin, 0)
        except Exception:
            pass  # Board could already have disconnected.
        try:
            board.shutdown()
        except Exception as error:
            print(f"Telemetrix shutdown warning: {error}")

    def set_servo_angle(self, channel, angle):
        """Write a 0-180 degree position to PCA9685 channel 0-15."""
        if self.board is None:
            return
        channel = int(channel)
        if not 0 <= channel <= 15:
            raise ValueError("PCA9685 channel must be between 0 and 15")
        angle = max(0, min(180, int(angle)))
        pulse_us = self.servo_min_us + (angle / 180.0) * (
            self.servo_max_us - self.servo_min_us
        )
        ticks = max(0, min(4095, int(pulse_us / 20000.0 * 4096)))
        register = 0x06 + 4 * channel
        self.board.i2c_write(
            PCA9685_ADDR, [register, 0, 0, ticks & 0xFF, (ticks >> 8) & 0xFF]
        )

    def update_motors(self, forwardSlider, turnSlider):
        """Original priority logic: strongest forward/back or turn wins."""
        if self.board is None:
            return
        forward = forwardSlider.value() - NEUTRAL
        turn = turnSlider.value() - NEUTRAL
        if abs(forward) > DEADZONE and abs(forward) >= abs(turn):
            if forward > 0:
                self.set_raw_motors(1, 0, 1, 0)
            else:
                self.set_raw_motors(0, 1, 0, 1)
        elif abs(turn) > DEADZONE:
            if turn > 0:
                self.set_raw_motors(1, 0, 0, 1)
            else:
                self.set_raw_motors(0, 1, 1, 0)
        else:
            self.stop_motors()

    def set_motor_speed(self, speed):
        """Set the speed for both motors (0-255)."""
        self.motor_speed = max(0, min(255, int(speed)))
        if self.board is not None:
            self.board.analog_write(EN1, self.motor_speed)
            self.board.analog_write(EN2, self.motor_speed)

    def set_raw_motors(self, l_in1, l_in2, r_in1, r_in2):
        if self.board is None:
            return
        for pin, value in zip(
            (IN1, IN2, IN3, IN4), (l_in1, l_in2, r_in1, r_in2)
        ):
            self.board.digital_write(pin, int(value))

        self.board.analog_write(EN1, self.motor_speed if (l_in1 or l_in2) else 0)
        self.board.analog_write(EN2, self.motor_speed if (r_in1 or r_in2) else 0)

    def stop_motors(self):
        if self.board is None:
            return
        self.set_raw_motors(0, 0, 0, 0)
