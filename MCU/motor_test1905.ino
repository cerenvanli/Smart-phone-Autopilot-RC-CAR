//This code written on 19.05.2024
//This code is to test the motor with speed percentages using the pwm



// Constants
const int MOTOR_PWM_CHANNEL1 = 0; // PWM channel for motor IN1
const int MOTOR_PWM_CHANNEL2 = 1; // PWM channel for motor IN2
const int MOTOR_PIN1 = 9;          // GPIO pin for motor IN1
const int MOTOR_PIN2 = 10;         // GPIO pin for motor IN2

void setup() {
  Serial.begin(9600); // Initialize serial communication

  // Setup PWM channels for motor
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
}

void loop() {
  int speed = 50; // Set the desired speed (0-100)

  // Map speed value (0-100) to PWM range (0-255)
  int pwmValue = map(speed, 0, 100, 0, 255);

  // Apply PWM value to motor
  analogWrite(MOTOR_PIN1, pwmValue);
  analogWrite(MOTOR_PIN2, 0); // Assuming one side of the motor is connected to ground for simplicity

  Serial.print("Speed set to: ");
  Serial.print(speed);
  Serial.print("%, PWM value: ");
  Serial.println(pwmValue);

  delay(1000); // Delay for demonstration purposes
}
