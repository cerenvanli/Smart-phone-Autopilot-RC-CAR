//This code is written and tested on 19.05.2024
//It tests the SoftAP with the servo


#include <WiFi.h>      // Include the WiFi library
#include <WiFiClient.h> // Include the WiFiClient library
#include <WiFiAP.h>    // Include the WiFiAP library
#include <ESP32Servo.h> // Include the ESP32Servo library

// WiFi credentials
const char *ssid = "esp-softAP";       // Name of Wi-Fi Network
const char *password = "12345678";   // Password of Wi-Fi Network

// Create an instance of the server
WiFiServer server(80);

// Constants
const int SERVO_PIN = 18; // GPIO pin for servo

// Create a servo object
Servo myServo;

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

  // Attach servo to pin
  myServo.attach(SERVO_PIN);
}

void loop() {
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {
    String request = client.readStringUntil('\r');  // Read the HTTP request until carriage return
    client.flush();  // Flush the client buffer
    Serial.println(request);  // Print the entire HTTP request

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
        int angle = angleStr.toInt();
        int speed = speedStr.toInt();
        
        Serial.print("Angle: ");
        Serial.println(angle);  // Print the angle
        
        Serial.print("Speed: ");
        Serial.println(speed);  // Print the speed

        // Control the servo
        steer(angle);
      }
    }
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





