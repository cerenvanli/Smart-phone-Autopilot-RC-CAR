#include <WiFi.h>

// WiFi credentials
const char *ssid = "esp-softAP";     // Name of Wi-Fi Network
const char *password = "12345678";   // Password of Wi-Fi Network

// IP address of the server
const char *serverIP = "192.168.4.1"; // IP address of the server

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
}

void loop() {
  // Send commands to the server at 10-second intervals
  sendCommand(70, 0);  // Angle 70, Speed 0
  delay(2000);
  sendCommand(120, 0); // Angle 120, Speed 0
  delay(2000);
  sendCommand(110, 0); // Angle 110, Speed 0
  delay(2000);
  sendCommand(55, 0);  // Angle 55, Speed 0
  delay(2000);
  sendCommand(150, 0); // Angle 150, Speed 0
  delay(2000);
}

void sendCommand(int angle, int speed) {
  // Convert angle and speed to strings
  String angleStr = String(angle);
  String speedStr = String(speed);

  // Pad angle with zeros to ensure it's 4 characters long
  while (angleStr.length() < 4) {
    angleStr = "0" + angleStr;
  }

  // Pad speed with zeros to ensure it's 3 characters long
  while (speedStr.length() < 3) {
    speedStr = "0" + speedStr;
  }

  // Construct the command string
  String commandStr = angleStr + speedStr + "00";

  // Send the command to the server
  Serial.print("Sending command: ");
  Serial.println(commandStr);
  WiFiClient client;
  if (!client.connect(serverIP, 80)) {
    Serial.println("Connection failed.");
    return;
  }

  client.print(String("GET /") + commandStr + " HTTP/1.1\r\n" +
               "Host: " + serverIP + "\r\n" +
               "Connection: close\r\n\r\n");

  delay(10);
  while (client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
  Serial.println();
  Serial.println("Command sent.");
}


