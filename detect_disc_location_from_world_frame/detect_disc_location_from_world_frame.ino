#include <HardwareSerial.h>

// Set up the Serial connection to the Camera (RX = Pin 16, TX = Pin 17)
HardwareSerial CameraSerial(2); 

// =======================================================
// 1. YOUR CALIBRATION PARAMETERS
// from the python script
// =======================================================
float p1 = 1.076769; // k*cos(theta)
float p2 = -0.021463; // k*sin(theta)
float tx = -301.325217; // Translation X
float ty = -162.999336; // Translation Y

// =======================================================
// 2. THE TRANSFORM FUNCTION
// =======================================================
void calculateDiscLocation(float pixel_x, float pixel_y, float &world_x, float &world_y) {
  // Applies the 2D Similarity Transform directly
  world_x = (p1 * pixel_x) - (p2 * pixel_y) + tx;
  world_y = (p2 * pixel_x) + (p1 * pixel_y) + ty;
}

void setup() {
  // Start the USB connection to your laptop for debugging
  Serial.begin(115200); 
  
  // Start the connection to the Camera (Make sure Camera D6 is wired to this ESP32's Pin 16)
  CameraSerial.begin(9600, SERIAL_8N1, 16, 17); 
  
  Serial.println("Robot Brain Online. Waiting for Camera Data...");
}

void loop() {
  // 1. Check if the camera has sent a new text string over the wire
  if (CameraSerial.available()) {
    
    // Read the text until the end of the line
    String incomingData = CameraSerial.readStringUntil('\n');
    incomingData.trim(); // Clean up any hidden spaces or carriage returns
    
    // 2. Did the camera lose the disc?
    if (incomingData == "NONE") {
      // The disc isn't visible. Do nothing, or tell the robot to stop moving.
      return; 
    }

    // 3. Variables to hold our extracted numbers
    int raw_pixel_x = 0;
    int raw_pixel_y = 0;
    
    // 4. THE MAGIC: Extract the numbers from the string "X:150,Y:120"
    if (sscanf(incomingData.c_str(), "X:%d,Y:%d", &raw_pixel_x, &raw_pixel_y) == 2) {
      
      // Variables to hold our final real-world coordinates
      float target_mm_x = 0;
      float target_mm_y = 0;
      
      // 5. RUN THE MATH ON THE FLY!
      calculateDiscLocation((float)raw_pixel_x, (float)raw_pixel_y, target_mm_x, target_mm_y);
      
      // 6. Print the results to your laptop to verify
      Serial.printf("Camera sees Pixels -> X:%d, Y:%d | ", raw_pixel_x, raw_pixel_y);
      Serial.printf("Robot Target -> X: %.1f mm, Y: %.1f mm\n", target_mm_x, target_mm_y);
      
      // =====================================================
      // NEXT STEP: INVERSE KINEMATICS
      // Send target_mm_x and target_mm_y into your IK function here!
      // moveRobotArm(target_mm_x, target_mm_y);
      // =====================================================
    }
  }
}