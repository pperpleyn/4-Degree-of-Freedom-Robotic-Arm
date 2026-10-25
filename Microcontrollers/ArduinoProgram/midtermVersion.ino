#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

// --- Bluetooth Pins ---
const int BT_RX = 2; // Connect to HC-05 TXD
const int BT_TX = 6; // Connect to HC-05 RXD (via voltage divider)
SoftwareSerial bluetooth(BT_RX, BT_TX);

// --- L298N Motor Driver Pins ---
const int in1 = 8;
const int in2 = 7;
const int in3 = 4;
const int in4 = 3;

// --- PCA9685 Servo Driver Setup ---
Adafruit_PWMServoDriver PWMservo = Adafruit_PWMServoDriver(0x40);

#define SERVO1_CHANNEL 0 // Base / Reach
#define SERVO2_CHANNEL 1 // Shoulder / Height
#define SERVO3_CHANNEL 2 // Elbow
#define SERVO4_CHANNEL 3 // Claw / Gripper

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);

  // Initialize Motor Pins
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  stopMotors();

  // Initialize PCA9685 Servo Driver
  PWMservo.begin();
  PWMservo.setPWMFreq(60);

  // Set Default Initial Arm Pose
  PWMservo.setPWM(SERVO1_CHANNEL, 0, 400);
  PWMservo.setPWM(SERVO2_CHANNEL, 0, 560);
  PWMservo.setPWM(SERVO3_CHANNEL, 0, 500);
  PWMservo.setPWM(SERVO4_CHANNEL, 0, 425);

  Serial.println("System Ready. Awaiting App Commands...");
}

void loop() {
  if (bluetooth.available() > 0) {
    char cmd = bluetooth.read();
    Serial.print("Command Received: ");
    Serial.println(cmd);
    processCommand(cmd);
  }
}

void processCommand(char c) {
  switch (c) {
    // --- Wheels Control ---
    case 'F': driveForward();  break;
    case 'B': driveBackward(); break;
    case 'L': turnLeft();      break;
    case 'R': turnRight();     break;
    case 'S': stopMotors();    break;

    // --- Arm Control (Triangle, X, Square, Circle) ---
    case 'T': // Triangle: Arm Forward and Up
      PWMservo.setPWM(SERVO1_CHANNEL, 0, 600);
      PWMservo.setPWM(SERVO2_CHANNEL, 0, 470);
      PWMservo.setPWM(SERVO3_CHANNEL, 0, 600);
      break;

    case 'X': // Cross/X: Arm Reverse and Down
      PWMservo.setPWM(SERVO1_CHANNEL, 0, 200);
      PWMservo.setPWM(SERVO2_CHANNEL, 0, 660);
      PWMservo.setPWM(SERVO3_CHANNEL, 0, 400);
      break;

    case 'Q': // Square: Arm Left
      PWMservo.setPWM(SERVO1_CHANNEL, 0, 550);
      break;

    case 'C': // Circle: Arm Right
      PWMservo.setPWM(SERVO1_CHANNEL, 0, 250);
      break;

    // --- Claw Control (Start / Pause) ---
    case 'O': // Start: Open Claw
      PWMservo.setPWM(SERVO4_CHANNEL, 0, 300);
      break;

    case 'P': // Pause: Close Claw
      PWMservo.setPWM(SERVO4_CHANNEL, 0, 580);
      break;

    default:
      break;
  }
}

// --- Wheel Movement Functions ---
void driveForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void driveBackward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}