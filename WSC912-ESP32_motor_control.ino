#include <AccelStepper.h>

// Motor 1
#define M1_STEP_PIN 4
#define M1_DIR_PIN  5

// Motor 2
#define M2_STEP_PIN 6
#define M2_DIR_PIN  7

// Motor 3
#define M3_STEP_PIN 15
#define M3_DIR_PIN  16

// Define interface type
// Type 1 = Driver with Step and Direction pins
#define MOTOR_INTERFACE_TYPE 1

// MOTOR OBJECTS
AccelStepper stepper1(MOTOR_INTERFACE_TYPE, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, M2_STEP_PIN, M2_DIR_PIN);
AccelStepper stepper3(MOTOR_INTERFACE_TYPE, M3_STEP_PIN, M3_DIR_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println("Yippee! system starting...");

  // ESP32 potentially too fast for DM556 drivers, so enforce minimum pulse width (in microseconds).
  // stepper1.setMinPulseWidth(20);
  // stepper2.setMinPulseWidth(20);
  // stepper3.setMinPulseWidth(20);

  stepper1.setCurrentPosition(0);  // TODO: need to draw line on teeth to set zero position
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0); 

  // --- MOTOR 1 SETTINGS ---
  stepper1.setMaxSpeed(800);      // Max speed in steps per second
  stepper1.setAcceleration(700);   // Acceleration in steps/second^2

  // --- MOTOR 2 SETTINGS ---
  stepper2.setMaxSpeed(1000);     
  stepper2.setAcceleration(500);

  // --- MOTOR 3 SETTINGS ---
  stepper3.setMaxSpeed(1000);     
  stepper3.setAcceleration(200);

  Serial.println("Motors configured. Starting motion...");
  // if motors make a whining noise, reduce acceleration (stepper motors can stall if accelerated too fast with a heavy driver)
}

void loop() {
  // .run() commands must be called as fast as possible
  // They check if it's time to take a step
  stepper1.run(); // Note that stepper 1 is positive up
  stepper2.run(); // Note that stepper 2 is positive anticlockwise
  stepper3.run(); // Note that stepper 3 is positive clockwise

  // --- INDIVIDUAL CONTROL LOGIC ---
  // this will be changed depending on what we need. Where do motors start? what do they do etc.
  // Will also need logic for recognising and picking up plastic component.
    
  
  // MOVE TO POT LOCATION
  if (stepper2.currentPosition() == 0) { // home location
    stepper2.moveTo(-2800);
    }
  if (stepper3.currentPosition() == 0) { // pot location
    stepper3.moveTo(2850);
    }
    // // move motor 1 left and right in loop
    // if (stepper1.distanceToGo() == 0) {
    //   // Flip direction
    //   if (stepper1.currentPosition() == 0) {
    //     stepper1.moveTo(1000);
    //   } else {
    //     // stepper1.moveTo(-1000);
    //   }
    // }

  // // move motor 2 left and right in loop
  // if (stepper2.distanceToGo() == 0) {
  //   // Flip direction
  //   if (stepper2.currentPosition() == 0) {
  //     stepper2.moveTo(1000);
  //   } else {
  //     // stepper2.moveTo(1000);
  //   }
  // }

  // // move motor 3 left and right in loop
  // if (stepper3.distanceToGo() == 0) {
  //   // Flip direction
  //   if (stepper3.currentPosition() == 0) {
  //     stepper3.moveTo(1000);
  //   } else {
  //     // stepper3.moveTo(1000);
  //   }
  // }


  
  // // Example: Check if Motor 1 has finished its journey
  // if (stepper1.distanceToGo() == 0) {
  //   // Flip direction: If we are at 2000, go to 0. If at 0, go to 2000.
  //   if (stepper1.currentPosition() == 100) {
  //     stepper1.moveTo(0);
  //   } else {
  //     stepper1.moveTo(100);
  //   }
  // }

  // // Example: Check if Motor 2 has finished
  // if (stepper1.distanceToGo() == 0) {
  //   // Just reverse direction relative to current position
  //   stepper1.moveTo(-stepper2.currentPosition());
  // }

  // Motor 3 will just stop when it reaches 4000 and stay there 
  // because we haven't given it new instructions in the loop.

  // expectation: Motor 1 and 2 will oscillate back and forth, while Motor 3 will move once and stop
}