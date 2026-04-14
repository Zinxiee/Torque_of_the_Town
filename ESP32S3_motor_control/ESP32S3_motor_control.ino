#include <AccelStepper.h>
#include <HardwareSerial.h>
#include "driver/ledc.h"
#include <math.h>

// ==========================================
//  HARDWARE PINS
// ==========================================
// Motor 1 (Prismatic Z-Axis)
#define M1_STEP_PIN 4
#define M1_DIR_PIN  5
#define LIMIT_PIN_Z 7

// Motor 2 (Shoulder - Rotary 1)
#define M2_STEP_PIN 42
#define M2_DIR_PIN  41

// Motor 3 (Elbow - Rotary 2)
#define M3_STEP_PIN 15
#define M3_DIR_PIN  16

#define LIMIT_PIN_XY 6 

// Camera Serial connection
HardwareSerial CameraSerial(2); 

// End effector servo
const int SERVO_PIN = 8;

#define MOTOR_INTERFACE_TYPE 1

AccelStepper stepper1(MOTOR_INTERFACE_TYPE, M1_STEP_PIN, M1_DIR_PIN); // prismatic joint
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, M2_STEP_PIN, M2_DIR_PIN); // revolute joint shoulder
AccelStepper stepper3(MOTOR_INTERFACE_TYPE, M3_STEP_PIN, M3_DIR_PIN); // revolute joint elbow

// ==========================================
//  STATE MACHINE
// ==========================================
// Every possible state the robot can be in. Only one state is active at a time.
enum RobotState {
  STATE_IDLE,
  STATE_LOCATING_DISC,       // Waiting for camera to find a disc
  STATE_MOVING_TO_DISC,      // Arm moving to disc XY position
  STATE_PICKUP_DESCEND,      // Z-axis lowering down to disc
  STATE_PICKUP_CLOSE,        // Gripper closing (instantaneous, but needs a small settle delay)
  STATE_PICKUP_ASCEND,       // Z-axis raising back up with disc to travel height
  STATE_DELIVER_MOVE_SAFE_POS,   // Arm moving to a safe intermediate XY before travelling to dropoff
  STATE_DELIVER_DROPOFF_HEIGHT,   // Z-axis lifting up to drop off height at safe XY
  STATE_DELIVER_MOVE_DROPOFF_POS,     // Arm moving to dropoff XY
  STATE_DELIVER_DROPOFF,       // Gripper opening to release disc
  STATE_RETURN_MOVE_SAFE,        // Arm returning to safe intermediate XY
  STATE_RETURN_TRAVEL_HEIGHT, // Z-axis descending back to travel height
  STATE_RETURN_HOME,         // Arm returning to home/observation position
};

RobotState currentState = STATE_IDLE;

// Timestamp used for non-blocking delays (e.g. gripper settle time)
unsigned long stateTimer = 0;

// ---- Calibration values (fill these in once tested) ----
const long Z_DISC_LEVEL     =   0 + 100; // TODO - test reducing this slightly (gripper appears to be abt 2mm from the ground. needs to be touching)
const long Z_TRAVEL_HEIGHT  = 2000;   // steps for Z safe travel height (~2cm from gnd)
const long Z_DROPOFF_HEIGHT = 12100;   // steps for Z at dropoff height
const long ELBOW_SAFETY = 100;

const float SAFE_XY_X = -148.0;      // Intermediate safe XY position before dropoff
const float SAFE_XY_Y =  190.0;
float DROPOFF_X = 46;    //50        // flipping station dropoff XY
float DROPOFF_Y = 181;   //182


const unsigned long GRIPPER_SETTLE_MS = 600; // Time to wait after closing/opening gripper // TODO - test this

// =======================================================
//  CAMERA CALIBRATION PARAMETERS
// from the python script
// =======================================================
float p1 = 0.915922; // k*cos(theta)
float p2 = 0.002437; // k*sin(theta)
float tx = -257.191511; // Translation X
float ty = -146.010561; // Translation Y

// ==========================================
//  ROBOT GEOMETRY & CONSTANTS
// ==========================================
const float L1 = 110.0; // length of shoulder to elbow, mm
const float L2 = 130.0; // length of elbow to end effector, mm
const float THETA1_HOME_DEG = 290.0; // shoulder joint home angle 
const float THETA2_HOME_DEG = 60.0; // elbow joint home angle (this was the cause of suffering)

// PHYSICAL JOINT LIMITS
// These physical limits of your joints are defined so the IK knows what is impossible.
const float THETA1_MIN_DEG = 90.0;     // Shoulder
const float THETA1_MAX_DEG = 295.0;   // Shoulder
const float THETA2_MIN_DEG = -150.0;  // Elbow maximum bend inwards (might need adjustment) #TODO
const float THETA2_MAX_DEG = 150.0;   // Elbow maximum bend outwards (might need adjustment) # TODO

// Motor scaling (with 1.8deg motor, 8x microstepping (1600 pulse/rev), 4:1 gear ratio) - essentially 1600 * 4 / 360
const float STEPS_PER_DEG = (200.0 * 8.0 * 4.0) / 360.0; 

// ==========================================
//  TARGET PICKUP ZONE
// ==========================================
const int NUM_CORNERS = 4;
// float pickupZone[NUM_CORNERS][2] = {
//   {-243.0, -82.0}, // Point 1: Bottom left
//   {-55.0, -86.0},  // Point 2: Bottom right
//   {-53.0, 82.0},   // Point 3: Top right
//   {-236.0, 82.0}   // Point 4: Top left
// };

void setup() {
  Serial.begin(115200);
  Serial.println("Yippee! system starting...");

  pinMode(LIMIT_PIN_Z, INPUT_PULLUP);
  pinMode(LIMIT_PIN_XY, INPUT_PULLUP);

  // ESP32 potentially too fast for DM556 drivers, so this prevents skipping steps // EXTRA perhaps decrease
  stepper1.setMinPulseWidth(20);
  stepper2.setMinPulseWidth(20);
  stepper3.setMinPulseWidth(20);

  // if motors make a whining noise, reduce acceleration (stepper motors can stall if accelerated too fast with a heavy driver)

  // --- MOTOR 1 SETTINGS (Prismatic) --- # SLOW! - max 50 RPM TODO
  stepper1.setMaxSpeed(16000);      // 8000 works with 1000 accel
  stepper1.setAcceleration(1000);

  // --- MOTOR 2 SETTINGS (Shoulder) ---
  stepper2.setMaxSpeed(12000);     // 6000
  stepper2.setAcceleration(5000); //1200

  // --- MOTOR 3 SETTINGS (Elbow) ---
  stepper3.setMaxSpeed(12000);     // 6000
  stepper3.setAcceleration(5000); //1200

  // Set initial home positions
  stepper2.setCurrentPosition(THETA1_HOME_DEG * STEPS_PER_DEG); // Note that stepper 2 is positive anticlockwise
  stepper3.setCurrentPosition(-THETA2_HOME_DEG * STEPS_PER_DEG); // Note that stepper 3 is positive clockwise

  // End-Effector config
  ledcAttach(SERVO_PIN, 50, 14);  // pin, 50Hz, 14-bit resolution
  // Fully Closed (0 degrees): 1000 microseconds
  // Neutral / Middle (90 degrees): 1500 microseconds
  // Fully Open (180 degrees): 2000 microseconds
  writeServoUs(600);   // start gripper fully closed to prevent collisions! (was gripperServo.write(5)) - TODO CHECK THIS WORKS


  // Start connection to the Camera (RX, TX)
  CameraSerial.setTimeout(20); 
  Serial.println("5");
  CameraSerial.begin(9600, SERIAL_8N1, 17, 18); // note slower serial for camera 
  Serial.println("Torquey is ...  A L I V E");

  Serial.println("Starting super fancy homing sequence...");
  homeRobot();
  
}

void loop() { // no delays or while loops allowed!
  // These must run constantly to generate the steps
  stepper1.run();
  stepper2.run();
  stepper3.run();

  // Route to the correct handler for whichever state we're in
  switch (currentState) {
  // PICK UP DISC PROCESS:
  // -> move prismatic joint down to position 0
  // -> close gripper
  // -> move prismatic joint up to position X

  // DELIVER DISC PROCESS: 
  // -> EE moves to lift up position
  // -> prismatic joint lifts up
  // -> EE moves to dropoff location
  // -> servo is opened and disc is dropped
  // -> EE moves to lift up position
  // -> prismatic joint drops down 

    case STATE_IDLE:
      // Do nothing. Transition triggered externally
      break;

    case STATE_LOCATING_DISC:
      handleLocatingDisc();
      break;

    case STATE_MOVING_TO_DISC:
      // Wait until BOTH arm joints have finished moving
      if (motorsIdle(false, true, true)) {
        // Queue the Z descent
        stepper1.moveTo(Z_DISC_LEVEL);
        openGripper();
        transitionTo(STATE_PICKUP_DESCEND);
      }
      break;

    case STATE_PICKUP_DESCEND:
      // Wait for Z-axis to reach disc level
      if (motorsIdle(true, false, false)) { // TODO MIGHT NEED THESE ALL TRUE
        // Z is down — now close the gripper
        closeGripper();  // fully closed (was gripperServo.write(5)) #TODO
        stateTimer = millis(); // Start settle timer
        transitionTo(STATE_PICKUP_CLOSE);
      }
      break;

    case STATE_PICKUP_CLOSE:
      // Give the gripper time to physically close before moving
      if (millis() - stateTimer >= GRIPPER_SETTLE_MS) {
        stepper1.moveTo(Z_TRAVEL_HEIGHT);
        transitionTo(STATE_PICKUP_ASCEND);
      }
      break;

    case STATE_PICKUP_ASCEND:
      if (motorsIdle(true, false, false)) {
        // move arms to safe XY before travelling to dropoff
        moveToPos(SAFE_XY_X, SAFE_XY_Y);
        transitionTo(STATE_DELIVER_MOVE_SAFE_POS);
      }
      break;
  
    case STATE_DELIVER_MOVE_SAFE_POS:
      if (motorsIdle(false, true, true)) {
        stepper1.moveTo(Z_DROPOFF_HEIGHT); 
        transitionTo(STATE_DELIVER_DROPOFF_HEIGHT);
      }
      break;

    case STATE_DELIVER_DROPOFF_HEIGHT:
      if (motorsIdle(true, false, false)) {
        moveToPos(DROPOFF_X, DROPOFF_Y); 
        transitionTo(STATE_DELIVER_MOVE_DROPOFF_POS);
      }
      break;

    case STATE_DELIVER_MOVE_DROPOFF_POS:
      if (motorsIdle(false, true, true)) {
        openGripper();   // fully open  (was gripperServo.write(90))
        stateTimer = millis();
        transitionTo(STATE_DELIVER_DROPOFF);
      }
      break;

    case STATE_DELIVER_DROPOFF:
      if (millis() - stateTimer >= GRIPPER_SETTLE_MS) {
        moveToPos(SAFE_XY_X, SAFE_XY_Y);  // retrace back through safe XY
        closeGripper();
        transitionTo(STATE_RETURN_MOVE_SAFE);
      }
      break;

    case STATE_RETURN_MOVE_SAFE:
      if (motorsIdle(false, true, true)) {
          stepper1.moveTo(Z_TRAVEL_HEIGHT);
          transitionTo(STATE_RETURN_TRAVEL_HEIGHT);
      }
      break;

    case STATE_RETURN_TRAVEL_HEIGHT:
      if (motorsIdle(true, false, false)) {
        stepper2.moveTo(THETA1_HOME_DEG * STEPS_PER_DEG);
        stepper3.moveTo(-THETA2_HOME_DEG * STEPS_PER_DEG + ELBOW_SAFETY);
        Serial.print("ELBOW HOME POS");
        Serial.println(-THETA2_HOME_DEG * STEPS_PER_DEG);
        transitionTo(STATE_RETURN_HOME);
      }
      break;

    case STATE_RETURN_HOME:
      if (motorsIdle(false, true, true)) {
        zHome();
        transitionTo(STATE_LOCATING_DISC);
      }
      break;
  }
}


// ==========================================
//  STATE MACHINE HELPERS
// ==========================================

// Call this whenever you want to move to a new state.
// Keeping transitions in one place makes debugging much easier.
void transitionTo(RobotState newState) {
  Serial.print("State: ");
  Serial.print(currentState);
  Serial.print(" -> ");
  Serial.println(newState);
  currentState = newState;
}

// Returns true when the selected motors have finished moving.
// Pass true for each motor axis you want to check.
bool motorsIdle(bool checkZ, bool checkShoulder, bool checkElbow) {
  if (checkZ       && stepper1.distanceToGo() != 0) return false;
  if (checkShoulder && stepper2.distanceToGo() != 0) return false;
  if (checkElbow   && stepper3.distanceToGo() != 0) return false;
  return true;
}

// =======================================================
//  SERVO
// =======================================================
void writeServoUs(uint32_t microseconds) {
  // Convert microseconds to a 16-bit duty cycle for a 50Hz signal
  // uint32_t dutyCycle = (microseconds * 65535) / 20000; 

  // Convert microseconds to a 14-bit duty cycle for a 50Hz signal (20,000 us period) - Max 14-bit value is 16383
  uint32_t dutyCycle = (microseconds * 16383) / 20000; // TODO - servo 14 bit change (check to see if servo opens)
  
  // Write the calculated duty cycle to the pin
  ledcWrite(SERVO_PIN, dutyCycle);
}

// =======================================================
//  CAMERA (XIAO ESP32S3 SENSE)
// =======================================================
void handleLocatingDisc() {
  if (!CameraSerial.available()) return; // EXTRA might be good to put a serial.print in this case
  
  String incomingData = "";
  
  // Read and discard everything in the buffer until we get to the very last line
  while (CameraSerial.available()) {
    incomingData = CameraSerial.readStringUntil('\n'); 
  }
  
  incomingData.trim(); // Clean up any hidden spaces or carriage returns
  
  // Did the camera lose the disc?
  if (incomingData == "NONE") {
    // The disc isn't visible. Do nothing, or tell the robot to stop moving. 
    return; 
  }

  // Variables to hold our extracted numbers and final real-world coordinates
  float target_mm_x = 0;
  float target_mm_y = 0;
  int raw_pixel_x = 0;
  int raw_pixel_y = 0;
  
  // Extract the numbers from the string "X:150,Y:120"
  if (sscanf(incomingData.c_str(), "X:%d,Y:%d", &raw_pixel_x, &raw_pixel_y) == 2) {
    calculateDiscLocation((float)raw_pixel_x, (float)raw_pixel_y, target_mm_x, target_mm_y);

    Serial.printf("Potential Disc Located -> X: %.1f mm, Y: %.1f mm\n", target_mm_x, target_mm_y);

    if (areValuesStable(target_mm_x, target_mm_y)) {
      Serial.printf("Moving to Disc Location -> X: %.1f mm, Y: %.1f mm\n", target_mm_x, target_mm_y);

      // Move the arms, then immediately start the descent once they arrive
      moveToPos(target_mm_x, target_mm_y);
      transitionTo(STATE_MOVING_TO_DISC);
    }
  }
}

//  2D SIMILARITY TRANSFORM
void calculateDiscLocation(float pixel_x, float pixel_y, float &world_x, float &world_y) {
  // Applies the 2D Similarity Transform directly
  world_x = (p1 * pixel_x) - (p2 * pixel_y) + tx;
  world_y = (p2 * pixel_x) + (p1 * pixel_y) + ty;
}

bool areValuesStable(float currentX, float currentY) {
  static float lastX = 0.0;       // Stores the previous X value
  static float lastY = 0.0;       // Stores the previous Y value
  static int count = 0;       // Tracks consecutive matches for both

  // Check if BOTH values are exactly the same as last time -  Use a small tolerance rather than exact equality for floats (within 3mm)
  if (fabs(currentX - lastX) < 3.0 && fabs(currentY - lastY) < 3.0) { // EXTRA - adjust if tolerance is too little
    count++;
  } else {
    lastX = currentX;
    lastY = currentY;
    count = 1;
  }

  // Check if they've both been stable for X in a row
  if (count >= 10) { // EXTRA: adjust for your camera update rate
    count = 0; // reset count
    return true;
  }
  return false;
}


// =======================================================
//  Gripper
// =======================================================
void closeGripper() {
  writeServoUs(600);  // fully closed (was gripperServo.write(5))
}

void openGripper() {
  writeServoUs(1600);   // fully open  (was gripperServo.write(90))
}


// ==========================================
//  INVERSE KINEMATICS FUNCTION
// ==========================================
void moveToPos(float x, float y) {
  // --- Reach Check ---
  float r = sqrt((x * x) + (y * y));
  float max_reach = L1 + L2;
  if (r > max_reach + 17.0) { Serial.println("Out of reach!"); return; } // EXTRA - can we change this hardcoded 17?
  if (r > max_reach) { x *= (max_reach/r); y *= (max_reach/r); }

  float r_sq = (x * x) + (y * y);
  float cos_theta2 = (r_sq - (L1 * L1) - (L2 * L2)) / (2.0 * L1 * L2);
  if (cos_theta2 > 1.0) cos_theta2 = 1.0;
  if (cos_theta2 < -1.0) cos_theta2 = -1.0;

  // --- Calculate angles in radians (Controls right or left elbow) ---
  // right elbow solution A
  float t2_rad_A = acos(cos_theta2); // acos always returns positive and so defaults to right elbow out
  float t1_rad_A = atan2(y, x) - atan2(L2 * sin(t2_rad_A), L1 + L2 * cos(t2_rad_A));
  float t1_deg_A = t1_rad_A * 180.0 / PI;
  float t2_deg_A = t2_rad_A * 180.0 / PI;

  // left elbow solution B
  float t2_rad_B = -acos(cos_theta2); // left elbow out
  float t1_rad_B = atan2(y, x) - atan2(L2 * sin(t2_rad_B), L1 + L2 * cos(t2_rad_B));
  float t1_deg_B = t1_rad_B * 180.0 / PI;
  float t2_deg_B = t2_rad_B * 180.0 / PI;

  // Normalize Shoulder to 0-360
  auto norm = [](float &a) { while(a<0) a+=360; while(a>=360) a-=360; };
  norm(t1_deg_A); norm(t1_deg_B);

  // Logic: Pick solution that doesn't hit a limit
  float final_t1, final_t2;
  if (checkLimits(t1_deg_A, t2_deg_A)) {
    final_t1 = t1_deg_A; final_t2 = t2_deg_A;
  } else if (checkLimits(t1_deg_B, t2_deg_B)){
    final_t1 = t1_deg_B; final_t2 = t2_deg_B;
  } else {
    Serial.println("CRITICAL WARNING: BOTH JOINT LIMITS HIT - IK IS UNSOLVABLE");
    transitionTo(STATE_LOCATING_DISC);
  }

  // Convert degrees to stepper motor steps
  long target_steps_2 = round(final_t1 * STEPS_PER_DEG); // Note that stepper 2 is positive anticlockwise
  long target_steps_3 = round(-final_t2 * STEPS_PER_DEG); // Note that stepper 3 is positive clockwise

  stepper2.moveTo(target_steps_2);
  stepper3.moveTo(target_steps_3);

  Serial.print("Target Angles - Shoulder: "); Serial.print(final_t1);
  Serial.print(" Elbow: "); Serial.println(final_t2);
}

bool checkLimits(float t1, float t2) {
  return (t1 >= THETA1_MIN_DEG && t1 <= THETA1_MAX_DEG && t2 >= THETA2_MIN_DEG && t2 <= THETA2_MAX_DEG);
}

void zHome() {
    // --- Prismatic joint (stepper1) ---
  stepper1.setSpeed(-600);
  while (digitalRead(LIMIT_PIN_Z) == HIGH) {
    stepper1.runSpeed();
  }
  stepper1.stop();
  stepper1.setCurrentPosition(0); // Note that stepper 1 is positive up
  // stepper1.moveTo(Z_TRAVEL_HEIGHT);
  stepper1.runToNewPosition(Z_TRAVEL_HEIGHT);
  Serial.println("Z Home!");
}

void homeRobot() { // EXTRA - this func might need additional configuration
  // Limit switch on LIMIT_PIN_Z is on the Z axis for prismatic joint
  // Limit switch on LIMIT_PIN_XY is mounted onto the SCARA and is triggered by the elbow joint

  // --- Prismatic joint (stepper1) ---
  stepper1.setSpeed(-600);
  while (digitalRead(LIMIT_PIN_Z) == HIGH) {
    stepper1.runSpeed();
  }
  stepper1.stop();
  stepper1.setCurrentPosition(0); // Note that stepper 1 is positive up
  // stepper1.moveTo(Z_TRAVEL_HEIGHT);
  stepper1.runToNewPosition(Z_TRAVEL_HEIGHT);


  // --- Elbow and Shoulder revolute joints ---
  // Is it a good or bad idea to move elbow say 90deg clockwise to avoid failure in homing (collision).
  stepper2.setSpeed(300); // Shoulder
  stepper3.setSpeed(-140); // Elbow
  while (digitalRead(LIMIT_PIN_XY) == HIGH) {
    stepper2.runSpeed();
    stepper3.runSpeed();
  }
  stepper2.stop();
  stepper3.stop();

  stepper2.setCurrentPosition(THETA1_HOME_DEG * STEPS_PER_DEG); // Note that stepper 2 is positive anticlockwise
  stepper3.setCurrentPosition(-THETA2_HOME_DEG * STEPS_PER_DEG); // Note that stepper 3 is positive clockwise
  stepper3.runToNewPosition(ELBOW_SAFETY);

  Serial.println("Homing Complete. Torquey is operational!");
  transitionTo(STATE_LOCATING_DISC);
}
