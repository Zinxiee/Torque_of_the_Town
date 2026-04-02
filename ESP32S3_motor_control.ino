#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include <math.h>

// ==========================================
//  HARDWARE PINS
// ==========================================
// Motor 1 (Prismatic Z-Axis)
#define M1_STEP_PIN 4
#define M1_DIR_PIN  5
#define LIMIT_PIN_2 10 // TODO THIS MUST BE CHANGED

// Motor 2 (Shoulder - Rotary 1)
#define M2_STEP_PIN 6
#define M2_DIR_PIN  7

// Motor 3 (Elbow - Rotary 2)
#define M3_STEP_PIN 15
#define M3_DIR_PIN  16

#define LIMIT_PIN_3 11 // TODO THIS MUST BE CHANGED

// Camera Serial connection
HardwareSerial CameraSerial(2); 

// End effector servo
Servo gripperServo;
const int SERVO_PIN = 8; // TODO miht need to be changed

bool locatingDisc = false; 
bool deliveringDisc = false;

#define MOTOR_INTERFACE_TYPE 1

AccelStepper stepper1(MOTOR_INTERFACE_TYPE, M1_STEP_PIN, M1_DIR_PIN); // prismatic joint
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, M2_STEP_PIN, M2_DIR_PIN); // revolute joint shoulder
AccelStepper stepper3(MOTOR_INTERFACE_TYPE, M3_STEP_PIN, M3_DIR_PIN); // revolute joint elbow

// =======================================================
//  CAMERA CALIBRATION PARAMETERS
// from the python script
// =======================================================
float p1 = 0.907631; // k*cos(theta)
float p2 = -0.017293; // k*sin(theta)
float tx = -259.497903; // Translation X
float ty = -140.493713; // Translation Y

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
const float THETA1_MAX_DEG = 290.0;   // Shoulder
const float THETA2_MIN_DEG = -140.0;  // Elbow maximum bend inwards (might need adjustment)
const float THETA2_MAX_DEG = 140.0;   // Elbow maximum bend outwards (might need adjustment)

// Motor scaling (with 1.8deg motor, 8x microstepping (1600 pulse/rev), 4:1 gear ratio) - essentially 1600 * 4 / 360
const float STEPS_PER_DEG = (200.0 * 8.0 * 4.0) / 360.0; 

// ==========================================
//  TARGET PICKUP ZONE
// ==========================================
const int NUM_CORNERS = 4;
float pickupZone[NUM_CORNERS][2] = {
  {-243.0, -82.0}, // Point 1: Bottom left
  {-55.0, -86.0},  // Point 2: Bottom right
  {-53.0, 82.0},   // Point 3: Top right
  {-236.0, 82.0}   // Point 4: Top left
};

void setup() {
  Serial.begin(115200);
  Serial.println("Yippee! system starting...");

  pinMode(LIMIT_PIN_2, INPUT_PULLUP);
  pinMode(LIMIT_PIN_3, INPUT_PULLUP);

  // ESP32 potentially too fast for DM556 drivers, so this prevents skipping steps // TODO perhaps decrease
  stepper1.setMinPulseWidth(20);
  stepper2.setMinPulseWidth(20);
  stepper3.setMinPulseWidth(20);

  // if motors make a whining noise, reduce acceleration (stepper motors can stall if accelerated too fast with a heavy driver)

  // --- MOTOR 1 SETTINGS (Prismatic) ---
  stepper1.setMaxSpeed(800);      
  stepper1.setAcceleration(700);  

  // --- MOTOR 2 SETTINGS (Shoulder) ---
  stepper2.setMaxSpeed(1000);     
  stepper2.setAcceleration(500);

  // --- MOTOR 3 SETTINGS (Elbow) ---
  stepper3.setMaxSpeed(1000);     
  stepper3.setAcceleration(200);

  // Set initial home positions
  stepper2.setCurrentPosition(THETA1_HOME_DEG * STEPS_PER_DEG); // Note that stepper 2 is positive anticlockwise
  stepper3.setCurrentPosition(-THETA2_HOME_DEG * STEPS_PER_DEG); // Note that stepper 3 is positive clockwise

  // End-Effector config
  gripperServo.setPeriodHertz(50);
  gripperServo.attach(SERVO_PIN, 500, 2400);
  gripperServo.write(55); // TODO Start angle of 55 might need to be changed

  // Start connection to the Camera (RX, TX)
  CameraSerial.begin(9600, SERIAL_8N1, 17, 18); // note slower serial for camera 
  Serial.println("Torquey is ...  A L I V E");

  Serial.println("Starting super fancy homing sequence...");
  homeRobot();
}

void loop() {
  // These must run constantly to generate the steps
  stepper1.run();
  stepper2.run();
  stepper3.run();

  if (locatingDisc) {
    locateThenMoveToDisc();
  }
  if (deliveringDisc) {
    pickUpDisc();
    deliverDisc();
  }
}


// =======================================================
//  CAMERA
// =======================================================
void locateThenMoveToDisc() {
  // 1. Check if the camera has sent a new text string over the wire
  if (CameraSerial.available()) {
    
    // Read the text until the end of the line
    String incomingData = CameraSerial.readStringUntil('\n'); // readStringUntil will stutter (and lose steps to) motors but should be fine as they shouldn't be moving for detection
    incomingData.trim(); // Clean up any hidden spaces or carriage returns
    
    // 2. Did the camera lose the disc?
    if (incomingData == "NONE") {
      // The disc isn't visible. Do nothing, or tell the robot to stop moving. 
      return; 
    }

    // 3. Variables to hold our extracted numbers
    int raw_pixel_x = 0;
    int raw_pixel_y = 0;
    
    // 4. Extract the numbers from the string "X:150,Y:120"
    if (sscanf(incomingData.c_str(), "X:%d,Y:%d", &raw_pixel_x, &raw_pixel_y) == 2) {
      
      // Variables to hold our final real-world coordinates
      float target_mm_x = 0;
      float target_mm_y = 0;
      
      // 5. RUN THE MATH ON THE FLY!
      calculateDiscLocation((float)raw_pixel_x, (float)raw_pixel_y, target_mm_x, target_mm_y);
      
      // 6. Print results to verify
      // Serial.printf("Camera sees Pixels -> X:%d, Y:%d | ", raw_pixel_x, raw_pixel_y);
      Serial.printf("Potential Disc Located -> X: %.1f mm, Y: %.1f mm\n", target_mm_x, target_mm_y);
      
      // if values remain the same then this is the location we want
      if (areValuesStable(target_mm_x, target_mm_y)) {
        Serial.printf("Moving to Disc Location -> X: %.1f mm, Y: %.1f mm\n", target_mm_x, target_mm_y);
        moveToPos(target_mm_x, target_mm_y);
        locatingDisc = false;
        deliveringDisc = true;
      }
    }
  }
}

//  2D SIMILARITY TRANSFORM
void calculateDiscLocation(float pixel_x, float pixel_y, float &world_x, float &world_y) {
  // Applies the 2D Similarity Transform directly
  world_x = (p1 * pixel_x) - (p2 * pixel_y) + tx;
  world_y = (p2 * pixel_x) + (p1 * pixel_y) + ty;
}

bool areValuesStable(int currentX, int currentY) { // there is a type mismatch here but should be fine as truncating (rounding) to nearest mm will be fine
  static int lastX = 0;       // Stores the previous X value
  static int lastY = 0;       // Stores the previous Y value
  static int count = 0;       // Tracks consecutive matches for both

  // Check if BOTH values are exactly the same as last time
  if ((currentX == lastX) && (currentY == lastY)) {
    count++;
  } else {
    // If EITHER value changed, update our records and reset the count
    lastX = currentX;
    lastY = currentY;
    count = 1; 
  }

  // Check if they've both been stable for 10 in a row // TODO ADJUST THIS ACCORDINGLY DEPENDING ON CAMERA!
  if (count >= 10) {
    // reset count
    count = 0; 
    return true;
  }
  return false;
}


// =======================================================
//  DISC OPERATIONS
// =======================================================
void pickUpDisc() {
  // performs pickup procedure

  // PROCESS:
  // -> move prismatic joint down to position 0
  // -> close gripper
  // -> move prismatic joint up to position X

  openGripper();
  if (stepper1.distanceToGo() == 0)
  stepper1.moveTo(0);
  closeGripper();
  stepper1.moveTo(@@); // TODO CALIBRATE THIS VALUE
}

void deliverDisc() {
  // performs the dropoff procedure. 

  // PROCESS: 
  // -> EE moves to lift up position
  // -> prismatic joint lifts up
  // -> EE moves to dropoff location
  // -> servo is opened and disc is dropped
  // -> EE moves to lift up position
  // -> prismatic joint drops down 

  Serial.println("Dropping off the Disc - Kachow!");
  moveToPos(-148, 190);
  stepper1.moveTo(2500); // TODO CALIBRATE THIS VALUE
  moveToPos(@@); // TODO DETERMINE X,Y COORDS OF DROPOFF LOCATION
  openGripper();
  moveToPos(-148, 190);
  stepper1.moveTo(0 + x); // TODO CALIBRATE THIS VALUE (i.e. 0 + x)
  resetPosition();
}

void resetPosition() {
  // is lift up pos in way of cam?  if so sol is to locate next disc whilst above flipping station or return to home position. - could add dual compatibility
  stepper1.moveTo(0 + 500); // Home position + ~2cm - TODO CALIBRATE THIS POS (End effector should be ~ 2cm from gnd)
  stepper2.moveTo(THETA1_HOME_DEG * STEPS_PER_DEG); // Home position
  stepper3.moveTo(-THETA2_HOME_DEG * STEPS_PER_DEG); // Home position
  if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
    locatingDisc = true;
    deliveringDisc = false;
  }
}

void closeGripper() {
  gripperServo.write(5);
}

void openGripper() {
  gripperServo.write(55); // can go to around 145
}


// ==========================================
//  INVERSE KINEMATICS FUNCTION
// ==========================================
void moveToPos(float x, float y) {
  // --- Reach Check ---
  float r = sqrt((x * x) + (y * y));
  float max_reach = L1 + L2;
  if (r > max_reach + 17.0) { Serial.println("Out of reach!"); return; } // TODO - can we change this hardcoded 17?
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

void homeRobot() { // TODO - this will need configuring - how fast and in what direction (-300) does each arm turn when resetting (are the limit pins the right way around?)
  // --- Prismatic joint (stepper1) ---
  stepper1.setSpeed(-300); // Adjust sign if it spins the wrong way
  while (digitalRead(LIMIT_PIN_2) == HIGH) {
    stepper1.runSpeed();
  }
  stepper1.stop();
  stepper1.setCurrentPosition(0); // Note that stepper 1 is positive up

  // --- Elbow and Shoulder revolute joints ---
  // Is it a good or bad idea to move elbow say 90deg clockwise to avoid failure in homing (collision). Can motors determine when steps are missed without encoder?
  stepper2.setSpeed(-300); // Shoulder - Adjust sign if it spins the wrong way
  stepper3.setSpeed(-100); // Elbow - Adjust sign if it spins the wrong way
  while (digitalRead(LIMIT_PIN_3) == HIGH) {
    stepper2.runSpeed();
    stepper3.runSpeed();
  }
  stepper2.stop();
  stepper3.stop();

  stepper2.setCurrentPosition(THETA1_HOME_DEG * STEPS_PER_DEG); // Note that stepper 2 is positive anticlockwise
  stepper3.setCurrentPosition(-THETA2_HOME_DEG * STEPS_PER_DEG); // Note that stepper 3 is positive clockwise

  Serial.println("Homing Complete. Torquey is operational!");
  // delay(1000) // delay?
}