#include <WiFi.h>            // Include the WiFi library
#include <WiFiClient.h>      // Include the WiFiClient library
#include <WiFiAP.h>          // Include the WiFiAP library
#include <ESP32Servo.h>      // Include the ESP32Servo library

#define analogWrite ledcWrite // Redefine analogWrite to use ledcWrite for PWM

// WiFi credentials
const char *ssid = "esp-softAP";       // Name of Wi-Fi Network
const char *password = "12345678";   // Password of Wi-Fi Network

// Create an instance of the server
WiFiServer server(80);

// Constants for servo motor
const int SERVO_PIN = 5; // GPIO pin for servo
int angle_degree = 90;
int speed_percentage;
// Create a servo object
Servo myServo;

// Constants for DC motor 
const int MOTOR_PWM_CHANNEL1 = 0; // PWM channel for motor IN1
const int MOTOR_PWM_CHANNEL2 = 1; // PWM channel for motor IN2
const int MOTOR_PIN1 = 19;        // GPIO pin for motor IN1
const int MOTOR_PIN2 = 18;        // GPIO pin for motor IN2
// Variables for PWM settings
int pwmFrequency = 1000; // PWM frequency in Hz
int pwmResolution = 8;   // PWM resolution in bits (0-255)

// Constants
const int FREQ = 1000;  // PWM frequency
const int RES = 8;      // PWM resolution


//Timing variables
unsigned long movedTime;
unsigned long startTime;
unsigned long receivedTime;
unsigned long decodedTime;
unsigned long motorControlTime;
void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate

  // Setup WiFi AP
  if (!WiFi.softAP(ssid, password)) {
    Serial.println("Failed to create SoftAP");
    while (1);
  }
  IPAddress myIP = WiFi.softAPIP(); // Get the IP address of the Soft AP
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  // Start the server
  server.begin();
  Serial.println("Server started");

  // Setup PWM channels for motor
  ledcSetup(MOTOR_PWM_CHANNEL1, pwmFrequency, pwmResolution); // Setup PWM for motor IN1 with frequency 1000Hz and resolution 8 bits
  ledcSetup(MOTOR_PWM_CHANNEL2, pwmFrequency, pwmResolution); // Setup PWM for motor IN2 with frequency 1000Hz and resolution 8 bits
  ledcAttachPin(MOTOR_PIN1, MOTOR_PWM_CHANNEL1); // Attach motor IN1 to PWM channel
  ledcAttachPin(MOTOR_PIN2, MOTOR_PWM_CHANNEL2); // Attach motor IN2 to PWM channel

  // Attach servo to pin
  myServo.attach(SERVO_PIN);        // Attach servo to PWM channel with min and max pulse width
}

void loop() {
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {
    startTime = micros(); // Start timing

    String request = client.readStringUntil('\r');  // Read the HTTP request until carriage return
    client.flush();  // Flush the client buffer
    Serial.println(request);  // Print the entire HTTP request

    receivedTime = micros(); // Time after receiving request

    if (request.startsWith("GET /")) {
      int firstSlashIndex = request.indexOf('/');
      int secondSpaceIndex = request.indexOf(' ', firstSlashIndex + 1);
      
      if (firstSlashIndex != -1 && secondSpaceIndex != -1) {
        String command = request.substring(firstSlashIndex + 1, secondSpaceIndex);  // Extract the command
        Serial.print("Received command: ");
        Serial.println(command);  // Print the command
        
        // Extract two 4-bit values from the command
        String angleStr = command.substring(0, 4);
        String speedStr = command.substring(4, 8);
        
        // Convert the two values to integers
        angle_degree = angleStr.toInt();
        speed_percentage = speedStr.toInt();
        
        Serial.print("Angle: ");
        Serial.println(angle_degree);  // Print the angle
        
        Serial.print("Speed: ");
        Serial.println(speed_percentage);  // Print the speed

        decodedTime = micros(); // Time after decoding command

      }
    }
      // Control the servo
        steer(angle_degree);
        movedTime = micros(); // Time after moving servo

        // Control the DC motor based on the speed
        controlMotor(speed_percentage);
        motorControlTime = micros(); // Time after controlling the motor

        // Log timing information
        Serial.print("Time to receive: ");
        Serial.print(receivedTime - startTime);
        Serial.println(" us");

        Serial.print("Time to decode: ");
        Serial.print(decodedTime - receivedTime);
        Serial.println(" us");

        Serial.print("Time to move servo: ");
        Serial.print(movedTime - decodedTime);
        Serial.println(" us");

        Serial.print("Time to control motor: ");
        Serial.print(motorControlTime - movedTime);
        Serial.println(" us");
  }
}

void steer(int angle) {
  // Ensure angle is within the desired range
  if (angle < 70) {
    angle = 70;
    Serial.println("Moving to:");
    Serial.println(angle);
  } else if (angle > 120) {
    angle = 120;
    Serial.println("Moving to:");
    Serial.println(angle);
  }
  
  // Set servo angle
  myServo.write(angle);
}

void controlMotor(int speed) {
  if (speed <= 100) {
    // Map speed value (0-100) to PWM range (0-255)
    int pwmValue = map(speed, 0, 100, 0, 255);

    // Apply PWM value to motor
    ledcWrite(MOTOR_PWM_CHANNEL1, pwmValue);
    ledcWrite(MOTOR_PWM_CHANNEL2, 0); // Assuming one side of the motor is connected to ground for simplicity
  } else {
    // Stop the motor for incorrect speed percentage
    ledcWrite(MOTOR_PWM_CHANNEL1, 0);
    ledcWrite(MOTOR_PWM_CHANNEL2, 0);
  }
}
