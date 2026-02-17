// Code for XIAO (Camera):
void setup() {
  Serial.begin(115200); // USB Debugging
  Serial1.begin(9600, SERIAL_8N1, D7, D6); // RX=D7, TX=D6
}

void loop() {
  Serial1.println("T:100,50"); // Send fake coordinates
  delay(1000);
}