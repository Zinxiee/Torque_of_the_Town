// TEST CODE FOR IK

#include <AccelStepper.h>

// --- PINS (From your foundation code) ---
#define M1_STEP_PIN 4
#define M1_DIR_PIN  5
#define M2_STEP_PIN 6
#define M2_DIR_PIN  7
#define M3_STEP_PIN 15
#define M3_DIR_PIN  16

AccelStepper stepper1(1, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepper2(1, M2_STEP_PIN, M2_DIR_PIN);
AccelStepper stepper3(1, M3_STEP_PIN, M3_DIR_PIN);

// --- CONSTANTS ---
const float STEPS_PER_REV = 200.0; // Based on 1.8 degree motor [cite: 37]
const float MICROSTEPS = 8.0;      // Based on your 1600 pulse/rev setting
const float GEAR_RATIO = 4.0;      // Based on your 4:1 Apiro ratio
const float STEPS_PER_DEG = (STEPS_PER_REV * MICROSTEPS * GEAR_RATIO) / 360.0;

void setup() {
  Serial.begin(115200);
  
  stepper2.setMaxSpeed(800);
  stepper2.setAcceleration(500);
  stepper3.setMaxSpeed(800);
  stepper3.setAcceleration(500);

  // Set minimum pulse width for DM556 (2.5us min) [cite: 240, 291]
  stepper2.setMinPulseWidth(20);
  stepper3.setMinPulseWidth(20);

  Serial.println("--- SCARA Diagnostic Mode ---");
  Serial.println("Commands:");
  Serial.println("1: Move Shoulder (M2) +90 degrees");
  Serial.println("2: Move Shoulder (M2) -90 degrees");
  Serial.println("3: Move Elbow (M3) +90 degrees");
  Serial.println("4: Move Elbow (M3) -90 degrees");
  Serial.println("0: Set current position as ZERO");
}

void loop() {
  stepper2.run();
  stepper3.run();

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == '1') {
      Serial.println("Moving Shoulder +90...");
      stepper2.move(90.0 * STEPS_PER_DEG);
    }
    if (cmd == '2') {
      Serial.println("Moving Shoulder -90...");
      stepper2.move(-90.0 * STEPS_PER_DEG);
    }
    if (cmd == '3') {
      Serial.println("Moving Elbow +90...");
      stepper3.move(90.0 * STEPS_PER_DEG);
    }
    if (cmd == '4') {
      Serial.println("Moving Elbow -90...");
      stepper3.move(-90.0 * STEPS_PER_DEG);
    }
    if (cmd == '0') {
      stepper2.setCurrentPosition(0);
      stepper3.setCurrentPosition(0);
      Serial.println("Positions Reset to Zero.");
    }
  }
}