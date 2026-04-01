#include <AccelStepper.h>
#include <math.h>

// ==========================================
// 1. HARDWARE PINS
// ==========================================
#define M1_STEP_PIN 4
#define M1_DIR_PIN  5

#define M2_STEP_PIN 6
#define M2_DIR_PIN  7
#define LIMIT_PIN_2 10 

#define M3_STEP_PIN 15
#define M3_DIR_PIN  16
#define LIMIT_PIN_3 11 

#define MOTOR_INTERFACE_TYPE 1

AccelStepper stepper1(MOTOR_INTERFACE_TYPE, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, M2_STEP_PIN, M2_DIR_PIN);
AccelStepper stepper3(MOTOR_INTERFACE_TYPE, M3_STEP_PIN, M3_DIR_PIN);

// ==========================================
// 2. ROBOT GEOMETRY & CONSTANTS
// ==========================================
const float L1 = 110.0; 
const float L2 = 130.0; 
const float THETA1_HOME_DEG = 290.0;  
const float THETA2_HOME_DEG = -60.0;  

// --- NEW: PHYSICAL JOINT LIMITS ---
// Define the safe physical limits of your joints so the IK knows what is impossible.
// Adjust these depending on where physical hard-stops or walls are defined!
const float THETA1_MIN_DEG = 90.0;     // E.g., Shoulder cannot go past East (0 deg)
const float THETA1_MAX_DEG = 290.0;   // E.g., Shoulder cannot go past home switch (290 deg)
const float THETA2_MIN_DEG = -140.0;  // E.g., Elbow maximum bend inwards (might need adjustment)
const float THETA2_MAX_DEG = 140.0;   // E.g., Elbow maximum bend outwards (might need adjustment)

const float STEPS_PER_DEG = (200.0 * 8.0 * 4.0) / 360.0; 

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
unsigned long pauseStartTime = 0;
bool isPaused = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Yippee! system starting...");

  pinMode(LIMIT_PIN_2, INPUT_PULLUP);
  pinMode(LIMIT_PIN_3, INPUT_PULLUP);

  stepper1.setMinPulseWidth(20);
  stepper2.setMinPulseWidth(20);
  stepper3.setMinPulseWidth(20);

  stepper1.setMaxSpeed(800);      
  stepper1.setAcceleration(700);  

  stepper2.setMaxSpeed(1000);     
  stepper2.setAcceleration(500);

  stepper3.setMaxSpeed(1000);     
  stepper3.setAcceleration(200);

  // Set initial home positions
  stepper2.setCurrentPosition(THETA1_HOME_DEG * STEPS_PER_DEG);
  stepper3.setCurrentPosition(-THETA2_HOME_DEG * STEPS_PER_DEG);
  
  Serial.println("Homing Complete. Beginning Trace in 3 seconds...");
  delay(3000); 
  
  isTracing = true;
  
  // Command the first move asynchronously
  Serial.print("Moving to Corner 1: ");
  moveToPos(pickupZone[0][0], pickupZone[0][1]);
}

void loop() {
  // These must run constantly to generate the steps
  stepper1.run();
  stepper2.run();
  stepper3.run();

  // --- NON-BLOCKING TRACING LOGIC ---
  if (isTracing) {
    // 1. Check if we just arrived at the target
    if (!isPaused && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0) {
      isPaused = true;
      pauseStartTime = millis(); // Start the timer
    }

    // 2. If we are paused, check if 2 seconds have passed
    if (isPaused && (millis() - pauseStartTime >= 2000)) {
      isPaused = false; // Unpause
      
      currentCorner++; // Move to next corner
      if (currentCorner >= NUM_CORNERS) {
        currentCorner = 0;
        Serial.println("Trace complete! Looping back to Corner 1...");
      }
      
      Serial.print("Moving to Corner ");
      Serial.print(currentCorner + 1);
      Serial.print(": ");
      
      // Send the new asynchronous target
      moveToPos(pickupZone[currentCorner][0], pickupZone[currentCorner][1]);
    }
  }
}

// ==========================================
// 4. SMART INVERSE KINEMATICS FUNCTION
// ==========================================
void moveToPos(float x, float y) {
  float r_sq = (x * x) + (y * y);
  float cos_theta2 = (r_sq - (L1 * L1) - (L2 * L2)) / (2.0 * L1 * L2);

  if (cos_theta2 > 1.0 || cos_theta2 < -1.0) {
    Serial.println("ERROR: Coordinate out of physical reach!");
    return; 
  }

  // Calculate Solution A (Elbow bends one way)
  float t2_rad_A = acos(cos_theta2);
  float t1_rad_A = atan2(y, x) - atan2(L2 * sin(t2_rad_A), L1 + L2 * cos(t2_rad_A));
  float t1_deg_A = t1_rad_A * 180.0 / PI;
  float t2_deg_A = t2_rad_A * 180.0 / PI;

  // Calculate Solution B (Elbow bends the opposite way)
  float t2_rad_B = -acos(cos_theta2);
  float t1_rad_B = atan2(y, x) - atan2(L2 * sin(t2_rad_B), L1 + L2 * cos(t2_rad_B));
  float t1_deg_B = t1_rad_B * 180.0 / PI;
  float t2_deg_B = t2_rad_B * 180.0 / PI;

  // Normalize angles to standard 0-360 or -180 to +180 ranges if needed
  // For simplicity, we just check against your physical limits
  bool valid_A = checkLimits(t1_deg_A, t2_deg_A);
  bool valid_B = checkLimits(t1_deg_B, t2_deg_B);

  float final_t1, final_t2;

  if (valid_A && !valid_B) {
    final_t1 = t1_deg_A; final_t2 = t2_deg_A;
  } else if (!valid_A && valid_B) {
    final_t1 = t1_deg_B; final_t2 = t2_deg_B;
  } else if (valid_A && valid_B) {
    // If BOTH are mathematically valid, pick Solution A by default 
    // (Or you could add logic here to pick the one closest to current position)
    final_t1 = t1_deg_A; final_t2 = t2_deg_A;
  } else {
    Serial.println("ERROR: Both solutions hit physical joint limits!");
    return;
  }

  // Convert to steps (Keeping the Step 3 inversion fix!)
  long target_steps_2 = round(final_t1 * STEPS_PER_DEG);
  long target_steps_3 = round(-final_t2 * STEPS_PER_DEG);

  stepper2.moveTo(target_steps_2);
  stepper3.moveTo(target_steps_3);

  Serial.print("Shoulder: "); Serial.print(final_t1);
  Serial.print(" | Elbow: "); Serial.println(final_t2);
}

// Helper function to check if angles are safe
bool checkLimits(float t1, float t2) {
  if (t1 < THETA1_MIN_DEG || t1 > THETA1_MAX_DEG) return false;
  if (t2 < THETA2_MIN_DEG || t2 > THETA2_MAX_DEG) return false;
  return true;
}