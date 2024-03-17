#include <Wire.h>
#include <VL6180X.h>

VL6180X sensor; // Create an instance of the VL6180X sensor object

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  Wire.begin(); // Initialize the I2C communication

  // Initialize the VL6180X sensor
  if (!sensor.begin(0x29)) { //if not worked will be tried with the 0x53 as the address
    Serial.println("Failed to initialize VL6180X sensor!"); // Print error message if initialization fails
    while (1); // Enter an infinite loop to halt the program
  }

  Serial.println("VL6180X sensor initialized successfully!"); // Print success message if initialization is successful
}

void loop() {
  int distance = sensor.readRangeSingleMillimeters(); // Read the distance in millimeters from the VL6180X sensor
  
  if (sensor.timeoutOccurred()) { // Check if a timeout occurred while reading from the sensor
    Serial.println("Failed to read from VL6180X sensor!"); // Print error message if a timeout occurred
  } else {
    Serial.print("Distance: "); // Print label for distance measurement
    Serial.print(distance); // Print the distance measurement in millimeters
    Serial.println(" mm"); // Print units (millimeters)
  }

  delay(1000); // Delay for 1 second before reading again
}
