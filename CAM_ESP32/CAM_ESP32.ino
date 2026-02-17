// Code for Main ESP32 (Robot):

#include <HardwareSerial.h>

HardwareSerial CameraSerial(2); // Use UART 2

void setup() {
  Serial.begin(115200);
  CameraSerial.begin(9600, SERIAL_8N1, 1, 44); 
}

void loop() {
  if (CameraSerial.available()) {
    String data = CameraSerial.readStringUntil('\n');
    Serial.print("Received from Camera: ");
    Serial.println(data);
  }
}