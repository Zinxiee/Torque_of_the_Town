#include <AccelStepper.h>
#include <math.h>

// ==========================================
// 1. HARDWARE PINS
// ==========================================
// Motor 1 (Prismatic Z-Axis)
#define M1_STEP_PIN 4
#define M1_DIR_PIN  5

// Motor 2 (Shoulder - Rotary 1)
#define M2_STEP_PIN 6
#define M2_DIR_PIN  7
#define LIMIT_PIN_2 10 // <-- Assign your shoulder limit switch here

// Motor 3 (Elbow - Rotary 2)
#define M3_STEP_PIN 15
#define M3_DIR_PIN  16
#define LIMIT_PIN_3 11 // <-- Assign your elbow limit switch here

#define MOTOR_INTERFACE_TYPE 1

AccelStepper stepper1(MOTOR_INTERFACE_TYPE, M1_STEP_PIN, M1_DIR_PIN); // prismatic
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, M2_STEP_PIN, M2_DIR_PIN); // rotary 1
AccelStepper stepper3(MOTOR_INTERFACE_TYPE, M3_STEP_PIN, M3_DIR_PIN); // rotary 2

// ==========================================
// 2. ROBOT GEOMETRY & CONSTANTS
// ==========================================
const float L1 = 110.0; // Length from Shoulder to Elbow
const float L2 = 130.0; // Length from Elbow to end-effector
const float THETA1_HOME_DEG = 290;  // Angle of shoulder when hitting limit
const float THETA2_HOME_DEG = -60;   // Angle of elbow when hitting limit

// Motor scaling (Assuming 1.8deg motor, 8x microstepping (1600 pulse/rev), 4:1 gear ratio)
const float STEPS_PER_DEG = (200.0 * 8.0 * 4.0) / 360.0; // TODO: Should this not just be 1600/360 (1600 steps per 1.8 deg / 360 deg)

// ==========================================
// 3. TARGET PICKUP ZONE ARRAY
// ==========================================
const int NUM_CORNERS = 4;
float pickupZone[NUM_CORNERS][2] = {
  {-243.0, -82.0}, // Point 1: Bottom left
  {-55.0, -86.0},  // Point 2: Bottom right
  {-53.0, 82.0},   // Point 3: Top right
  {-236.0, 82.0}   // Point 4: Top left
};

int currentCorner = 0;
bool isTracing = false; 

void setup() {
  Serial.begin(115200);
  Serial.println("Yippee! system starting...");

  pinMode(LIMIT_PIN_2, INPUT_PULLUP);
  pinMode(LIMIT_PIN_3, INPUT_PULLUP);

  // ESP32 potentially too fast for DM556 drivers. Uncommenting this prevents skipped steps.
  stepper1.setMinPulseWidth(20);
  stepper2.setMinPulseWidth(20);
  stepper3.setMinPulseWidth(20);

  // --- MOTOR 1 SETTINGS (Prismatic - Idle for now) ---
  stepper1.setMaxSpeed(800);      
  stepper1.setAcceleration(700);  

  // --- MOTOR 2 SETTINGS (Shoulder) ---
  stepper2.setMaxSpeed(1000);     
  stepper2.setAcceleration(500);

  // --- MOTOR 3 SETTINGS (Elbow) ---
  stepper3.setMaxSpeed(1000);     
  stepper3.setAcceleration(200);

  Serial.println("Motors configured. Starting homing sequence...");
  homeRobot();
  
Serial.println("Homing Complete. Moving to Bottom Right in 2 seconds...");
  delay(2000); 
  
  // Move to Bottom Right (Point 2)
  moveToPos(pickupZone[1][0], pickupZone[1][1]);
  runToTarget(); // Force the ESP32 to finish the move before continuing
  
  Serial.println("Reached Bottom Right! Waiting 2 seconds...");
  delay(2000);

  // Move to Top Left (Point 4)
  Serial.println("Moving to Top Left...");
  moveToPos(pickupZone[3][0], pickupZone[3][1]);
  runToTarget(); // Force the ESP32 to finish the move
  
  Serial.println("Trace Complete!");
}

void loop() {
  // .run() commands must be called as fast as possible
  stepper1.run(); // positive up (prismatic)
  stepper2.run(); // positive anticlockwise (rotary)
  stepper3.run(); // positive clockwise (rotary)

  // --- TRACING LOGIC ---
//   if (isTracing && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
    
//     delay(1500); // Pause for 1.5 seconds at the target
    
//     currentCorner++;
//     if (currentCorner >= NUM_CORNERS) {
//       currentCorner = 0;
//       Serial.println("Trace complete! Looping back to Corner 1...");
//     } else {
//       Serial.print("Moving to Corner ");
//       Serial.println(currentCorner + 1);
//     }
    
//     moveToPos(pickupZone[currentCorner][0], pickupZone[currentCorner][1]);
//   }
}

// ==========================================
// BLOCKING MOVE (till locations are reached)
// ==========================================
void runToTarget() {
  // This traps the code here, rapidly stepping the motors until both reach 0 distance to go
  while (stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
    stepper2.run();
    stepper3.run();
  }
}

// ==========================================
// 4. INVERSE KINEMATICS FUNCTION
// ==========================================
void moveToPos(float x, float y) {
  float r_sq = (x * x) + (y * y);
  float cos_theta2 = (r_sq - (L1 * L1) - (L2 * L2)) / (2.0 * L1 * L2);

  if (cos_theta2 > 1.0 || cos_theta2 < -1.0) {
    Serial.println("ERROR: Target coordinate is out of physical reach!");
    return; 
  }

  // Calculate angles in radians (Controls right or left elbow)
  float theta2_rad = acos(cos_theta2); // acos always returns positive and so defaults to right elbow
  float theta1_rad = atan2(y, x) - atan2(L2 * sin(theta2_rad), L1 + L2 * cos(theta2_rad));

  // Convert radians to degrees
  float theta1_deg = theta1_rad * 180.0 / PI;
  float theta2_deg = theta2_rad * 180.0 / PI;

  // Convert degrees to stepper motor steps
  long target_steps_2 = round(theta1_deg * STEPS_PER_DEG);
  
  // NOTE: stepper3 is physically positive clockwise, but math assumes CCW is positive.
  // We multiply by -1 to invert the direction and keep the math accurate.
  long target_steps_3 = round(-theta2_deg * STEPS_PER_DEG);

  // Command the rotary motors (stepper 1 stays at 0)
  stepper2.moveTo(target_steps_2);
  stepper3.moveTo(target_steps_3);

  Serial.print("Target Angles - Shoulder: "); Serial.print(theta1_deg);
  Serial.print(" Elbow: "); Serial.println(theta2_deg);
}

// ==========================================
// 5. HOMING FUNCTION
// ==========================================
void homeRobot() {
  // Home Elbow (stepper3)
  // stepper3.setSpeed(-300); // Adjust sign if it spins the wrong way
  // while (digitalRead(LIMIT_PIN_3) == HIGH) {
  //   stepper3.runSpeed();
  // }
  // stepper3.stop();
  stepper3.setCurrentPosition(-THETA2_HOME_DEG * STEPS_PER_DEG); // negative because it is physically +ve clockwise but math assumes +ve CCW

  // Home Shoulder (stepper2)
  // stepper2.setSpeed(-300); // Adjust sign if it spins the wrong way
  // while (digitalRead(LIMIT_PIN_2) == HIGH) {
  //   stepper2.runSpeed();
  // }
  // stepper2.stop();
  stepper2.setCurrentPosition(THETA1_HOME_DEG * STEPS_PER_DEG);
}
