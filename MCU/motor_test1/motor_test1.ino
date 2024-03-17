#include <Wire.h>
#include <Adafruit_VL6180X.h>

// Pin definitions
#define MOTOR1_PWM_PIN 25    // Motor 1 PWM pin
#define MOTOR1_DIR_PIN1 26   // Motor 1 direction pin 1
#define MOTOR1_DIR_PIN2 27   // Motor 1 direction pin 2
#define MOTOR2_PWM_PIN 32    // Motor 2 PWM pin
#define MOTOR2_DIR_PIN1 33   // Motor 2 direction pin 1
#define MOTOR2_DIR_PIN2 34   // Motor 2 direction pin 2

// TOF sensor address
#define VL6180X_ADDRESS 0x29

Adafruit_VL6180X vl = Adafruit_VL6180X();

void setup() {
  Serial.begin(115200);  // Start serial communication
  Wire.begin();           // Initialize I2C communication

  // Initialize the TOF sensor
  if (!vl.begin()) {
    Serial.println("TOF sensor not found!");
    while (1);
  }
  Serial.println("TOF sensor successfully initialized!");

  // Set motor pins as outputs
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR1_DIR_PIN1, OUTPUT);
  pinMode(MOTOR1_DIR_PIN2, OUTPUT);
  pinMode(MOTOR2_PWM_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN1, OUTPUT);
  pinMode(MOTOR2_DIR_PIN2, OUTPUT);
}

void loop() {
  // Measure distance
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  // Move forward if no obstacle detected
  if (status != VL6180X_ERROR_NONE || range > 100) {
    moveForward();
  } else { // Stop if obstacle detected
    stopMotors();
  }

  delay(100);
}

// Function to move motors forward
void moveForward() {
  digitalWrite(MOTOR1_DIR_PIN1, HIGH);
  digitalWrite(MOTOR1_DIR_PIN2, LOW);
  analogWrite(MOTOR1_PWM_PIN, 255); // Run at full speed

  digitalWrite(MOTOR2_DIR_PIN1, HIGH);
  digitalWrite(MOTOR2_DIR_PIN2, LOW);
  analogWrite(MOTOR2_PWM_PIN, 255); // Run at full speed
}

// Function to stop motors
void stopMotors() {
  digitalWrite(MOTOR1_DIR_PIN1, LOW);
  digitalWrite(MOTOR1_DIR_PIN2, LOW);
  analogWrite(MOTOR1_PWM_PIN, 0); // Stop motor 1

  digitalWrite(MOTOR2_DIR_PIN1, LOW);
  digitalWrite(MOTOR2_DIR_PIN2, LOW);
  analogWrite(MOTOR2_PWM_PIN, 0); // Stop motor 2
}
