#include <AccelStepper.h>
#include <math.h>

// ==========================================
// 1. HARDWARE PINS (ESP32-S3)
// ==========================================
// Replace these with your actual ESP32-S3 pins
#define STEP_PIN_1  6
#define DIR_PIN_1   7
#define LIMIT_PIN_1 6

#define STEP_PIN_2  15
#define DIR_PIN_2   16
#define LIMIT_PIN_2 9

// Initialize AccelStepper for step/dir drivers (like A4988, TMC2209)
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

// ==========================================
// 2. ROBOT GEOMETRY & MOTOR SPECS
// ==========================================
// USER INPUT REQUIRED: Enter the lengths of your Apiro arms in mm
const float L1 = 110.0; // Length from Shoulder to Elbow
const float L2 = 130.0; // Length from Elbow to end-effector

// USER INPUT REQUIRED: Calculate your steps per degree
const float STEPS_PER_REV = 360.0 / 1.8; // Standard NEMA 17 (360 / angle per step)
const float MICROSTEPS = 800.0 / STEPS_PER_REV;     // Your driver's microstepping setting (Pulse per rev / STEPS_PER_REV)
const float GEAR_RATIO_1 = 32.0;   // Apiro gearbox ratio for Joint 1
const float GEAR_RATIO_2 = 32.0;   // Apiro gearbox ratio for Joint 2

const float STEPS_PER_DEG_1 = (STEPS_PER_REV * MICROSTEPS * GEAR_RATIO_1) / 360.0;
const float STEPS_PER_DEG_2 = (STEPS_PER_REV * MICROSTEPS * GEAR_RATIO_2) / 360.0;

// USER INPUT REQUIRED: Your measured limit switch angles in degrees
const float THETA1_HOME_DEG = 107.8019;  // Angle of shoulder when hitting limit
const float THETA2_HOME_DEG = 40.1981; // Angle of elbow when hitting limit

void setup() {
  Serial.begin(115200);
  
  pinMode(LIMIT_PIN_1, INPUT_PULLUP);
  pinMode(LIMIT_PIN_2, INPUT_PULLUP);

  // Set max speeds and accelerations (in steps per second)
  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(1000);
  stepper2.setMaxSpeed(2000);
  stepper2.setAcceleration(1000);

  Serial.println("Starting Homing Sequence...");
  homeRobot();
  Serial.println("Homing Complete.");

  // Example: Move to the center of your pickup zone
  // Average of X bounds: ~ -146, Average of Y bounds: ~ 0
  Serial.println("Moving to pickup zone...");
  moveToPos(-146.0, 0.0);
}

void loop() {
  // Required to keep steppers moving to their target positions
  stepper1.run();
  stepper2.run();
}

// ==========================================
// 3. INVERSE KINEMATICS FUNCTION
// ==========================================
void moveToPos(float x, float y) {
  // accepts X,Y location in global reference frame
  // 1. Calculate the distance squared from origin to target
  float r_sq = (x * x) + (y * y);

  // 2. Law of Cosines to find the elbow angle (Theta 2)
  float cos_theta2 = (r_sq - (L1 * L1) - (L2 * L2)) / (2.0 * L1 * L2);

  // Check if the target is physically reachable
  if (cos_theta2 > 1.0 || cos_theta2 < -1.0) {
    Serial.println("ERROR: Target coordinate is out of reach!");
    return; 
  }

  // Calculate Theta 2 in radians
  // NOTE: acos() returns a positive value. If your arm needs to bend the opposite way 
  // to reach the left-side pickup zone, you may need to use: float theta2_rad = -acos(cos_theta2);
  float theta2_rad = acos(cos_theta2);

  // 3. Calculate the shoulder angle (Theta 1) in radians
  float theta1_rad = atan2(y, x) - atan2(L2 * sin(theta2_rad), L1 + L2 * cos(theta2_rad));

  // 4. Convert radians to degrees
  float theta1_deg = theta1_rad * 180.0 / PI;
  float theta2_deg = theta2_rad * 180.0 / PI;

  // 5. Convert degrees to stepper motor steps
  long target_steps_1 = round(theta1_deg * STEPS_PER_DEG_1);
  long target_steps_2 = round(theta2_deg * STEPS_PER_DEG_2);

  // 6. Command the motors
  stepper1.moveTo(target_steps_1);
  stepper2.moveTo(target_steps_2);

  Serial.print("Target Angles - J1: "); Serial.print(theta1_deg);
  Serial.print(" J2: "); Serial.println(theta2_deg);
}

// ==========================================
// 4. HOMING FUNCTION
// ==========================================
void homeRobot() { // TODO ADD IN LIMIT SWITCH
  // Move Joint 2 (Elbow) to limit switch first to avoid collisions
  // stepper2.setSpeed(-500); // Adjust sign based on motor wiring direction
  // while (digitalRead(LIMIT_PIN_2) == HIGH) { 
  //   stepper2.runSpeed();
  // }
  stepper2.stop();
  // THIS IS THE CRITICAL OFFSET: We tell the driver exactly what angle it is currently at
  stepper2.setCurrentPosition(THETA2_HOME_DEG * STEPS_PER_DEG_2);

  // Move Joint 1 (Shoulder) to limit switch
  stepper1.setSpeed(-500); // Adjust sign based on motor wiring direction
  // while (digitalRead(LIMIT_PIN_1) == HIGH ) { 
  //   stepper1.runSpeed();
  // }
  stepper1.stop();
  // Set the shoulder offset
  stepper1.setCurrentPosition(THETA1_HOME_DEG * STEPS_PER_DEG_1);
}
