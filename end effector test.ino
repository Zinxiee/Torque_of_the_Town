#include <ESP32Servo.h>

Servo gripperServo;

const int SERVO_PIN = 8;

// Start angle
int servoAngle = 90;

void setup() {
  Serial.begin(115200);
  delay(1000);

  gripperServo.setPeriodHertz(50);
  gripperServo.attach(SERVO_PIN, 500, 2400);

  gripperServo.write(servoAngle);

  Serial.println("Servo slider test");
  Serial.println("Send a value from 0 to 180 in Serial Monitor.");
  Serial.println("Example: 45");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      int newAngle = input.toInt();

      if (newAngle >= 0 && newAngle <= 180) {
        servoAngle = newAngle;
        gripperServo.write(servoAngle);

        Serial.print("Angle set to: ");
        Serial.println(servoAngle);
      } else {
        Serial.println("Enter an angle between 0 and 180");
      }
    }
  }
}