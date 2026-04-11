#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ==========================================
// 1. WIFI NETWORK
// ==========================================
const char* ssid = "RobotVision"; 
const char* password = "TorqueOfTheTown";

// ==========================================
// 2. BLOB DETECTION & AXIS CALIBRATION
// ==========================================
const int WHITE_THRESHOLD = 250; 
const int MIN_PIXELS = 50;

// Clustering settings
#define MAX_BLOBS 5
const int MERGE_DISTANCE = 5; // Max gap (in pixels) between white pixels to be considered the same blob

// Debug mode to see binary mask (black and white thresholding)
const bool DEBUG_MASK_MODE = true;

// --- AXIS TRANSFORMATION TOGGLES ---
const bool SWAP_XY_AXES = true;       // True: X becomes Y, Y becomes X
const bool INVERT_X_DIRECTION = true; // True: Flips the X scale backwards
const bool INVERT_Y_DIRECTION = true; // True: Flips the Y scale backwards

// ==========================================
// 3. CROP BOX (REGION OF INTEREST)
// ==========================================
// Max Width is 320, Max Height is 240.
const int CROP_X_MIN = 40;  // Ignore the left 40 pixels
const int CROP_X_MAX = 280; // Ignore the right 40 pixels
const int CROP_Y_MIN = 5;   // Ignore the top 5 pixels
const int CROP_Y_MAX = 240; // Ignore the bottom 0 pixels

// ==========================================
// XIAO ESP32S3 SENSE PINOUT
// ==========================================
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// ==========================================
// DATA STRUCTURES
// ==========================================
struct TrackedBlob {
  long sum_x;
  long sum_y;
  int count;
  int min_x, max_x, min_y, max_y;
};

struct BlobResult {
  bool found;
  int raw_x;
  int raw_y;
  int final_x;
  int final_y;
};

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

// ==========================================
// CORE VISION ALGORITHM (Clustering + Positional Lock)
// ==========================================
BlobResult detectBlob(camera_fb_t * fb) {
  BlobResult result = {false, 0, 0, 0, 0};
  
  TrackedBlob blobs[MAX_BLOBS];
  int blob_count = 0;

  // --- PASS 1: Scan and Group Pixels ---
  for (int y = CROP_Y_MIN; y < CROP_Y_MAX; y++) {
    for (int x = CROP_X_MIN; x < CROP_X_MAX; x++) {
      int index = (y * fb->width) + x;
      
      if (fb->buf[index] > WHITE_THRESHOLD) {
        bool matched_to_existing_blob = false;
        
        // Check if this pixel is near an existing blob's bounding box
        for (int b = 0; b < blob_count; b++) {
          if (x >= blobs[b].min_x - MERGE_DISTANCE && x <= blobs[b].max_x + MERGE_DISTANCE &&
              y >= blobs[b].min_y - MERGE_DISTANCE && y <= blobs[b].max_y + MERGE_DISTANCE) {
              
            blobs[b].sum_x += x;
            blobs[b].sum_y += y;
            blobs[b].count++;
            
            if (x < blobs[b].min_x) blobs[b].min_x = x;
            if (x > blobs[b].max_x) blobs[b].max_x = x;
            if (y < blobs[b].min_y) blobs[b].min_y = y;
            if (y > blobs[b].max_y) blobs[b].max_y = y;
            
            matched_to_existing_blob = true;
            break; 
          }
        }

        // If it isn't close to any existing blob, start a new one
        if (!matched_to_existing_blob && blob_count < MAX_BLOBS) {
          blobs[blob_count].sum_x = x;
          blobs[blob_count].sum_y = y;
          blobs[blob_count].count = 1;
          blobs[blob_count].min_x = x;
          blobs[blob_count].max_x = x;
          blobs[blob_count].min_y = y;
          blobs[blob_count].max_y = y;
          blob_count++;
        }
        
        if (DEBUG_MASK_MODE) fb->buf[index] = 255;
      } else {
        if (DEBUG_MASK_MODE) fb->buf[index] = 0;
      }
    }
  }

  // --- PASS 2: Pick the Best Target (Positional Lock) ---
  int best_blob_index = -1;

  for (int b = 0; b < blob_count; b++) {
    if (blobs[b].count > MIN_PIXELS) {
      best_blob_index = b;
      break; // Stop looking! First valid blob is our target.
    }
  }

  // --- 3. CALCULATE OUTPUT & AXIS TRANSFORMATION ---
  if (best_blob_index != -1) {
    result.found = true;
    result.raw_x = blobs[best_blob_index].sum_x / blobs[best_blob_index].count;
    result.raw_y = blobs[best_blob_index].sum_y / blobs[best_blob_index].count;

    result.final_x = result.raw_x;
    result.final_y = result.raw_y;

    if (SWAP_XY_AXES) {
      result.final_x = result.raw_y;
      result.final_y = result.raw_x;
    }

    if (INVERT_X_DIRECTION) {
      int max_x = SWAP_XY_AXES ? fb->height : fb->width;
      result.final_x = (max_x - 1) - result.final_x;
    }
    if (INVERT_Y_DIRECTION) {
      int max_y = SWAP_XY_AXES ? fb->width : fb->height;
      result.final_y = (max_y - 1) - result.final_y;
    }
  }

  return result;
}

// ==========================================
// WEB SERVER HANDLER (Runs when browser is open)
// ==========================================
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK) return res;

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    // Call the unified detection algorithm
    BlobResult blob = detectBlob(fb);

    // --- DRAW THE CROP BOX ---
    for(int i = CROP_X_MIN; i < CROP_X_MAX; i++) {
        fb->buf[CROP_Y_MIN * fb->width + i] = 100; 
        fb->buf[(CROP_Y_MAX-1) * fb->width + i] = 100;
    }
    for(int i = CROP_Y_MIN; i < CROP_Y_MAX; i++) {
        fb->buf[i * fb->width + CROP_X_MIN] = 100;
        fb->buf[i * fb->width + (CROP_X_MAX-1)] = 100;
    }

    // --- DRAW CROSSHAIR & SEND SERIAL DATA ---
    if (blob.found) {
      // Draw crosshair at the RAW optical location
      for(int i=-10; i<=10; i++){
        if(blob.raw_x+i>=0 && blob.raw_x+i<fb->width) fb->buf[blob.raw_y*fb->width + (blob.raw_x+i)] = 0; 
        if(blob.raw_y+i>=0 && blob.raw_y+i<fb->height) fb->buf[(blob.raw_y+i)*fb->width + blob.raw_x] = 0; 
      }

      Serial1.printf("X:%d,Y:%d\n", blob.final_x, blob.final_y);
      Serial.printf("[Stream] Raw: %d,%d | Output -> X:%d, Y:%d\n", blob.raw_x, blob.raw_y, blob.final_x, blob.final_y);
    } else {
      Serial1.println("NONE");
    }

    // COMPRESS & STREAM OVER WIFI
    bool jpeg_converted = frame2jpg(fb, 20, &_jpg_buf, &_jpg_buf_len); 
    esp_camera_fb_return(fb); 
    fb = NULL;

    if(!jpeg_converted){ res = ESP_FAIL; }

    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    
    if(_jpg_buf){ free(_jpg_buf); _jpg_buf = NULL; } 
    if(res != ESP_OK) break;
  }
  return res;
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // Start UART connection to Main ESP32 (D7=RX, D6=TX)
  Serial1.begin(9600, SERIAL_8N1, D7, D6);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM; config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM; config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM; config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  
  config.pixel_format = PIXFORMAT_GRAYSCALE; 
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.frame_size = FRAMESIZE_QVGA; 
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed! 0x%x\n", err);
    return;
  }

  // --- HARDWARE EXPOSURE TUNING ---
  // Software flipping of camera hardware
  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 0);
  
  // Tweak sensor to favor dark environments to isolate bright spots
  s->set_brightness(s, -2);     // Lower overall brightness (-2 to 2)
  s->set_contrast(s, 2);        // High contrast makes thresholding easier (-2 to 2)
  s->set_aec2(s, 1);            // Enable auto-exposure DSP
  s->set_ae_level(s, -2);       // Shift auto-exposure target to be darker

  // --- CREATE THE WIFI NETWORK ---
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  startCameraServer();

  Serial.println("=========================================");
  Serial.println("HEYO CAMERA IS OPERATIONAL, TRACKING IS LIVE");
  Serial.print("Network Created! Connect to: "); Serial.println(ssid);
  Serial.print("Stream Video at: http://"); Serial.println(IP);
  Serial.println("Coordinates are streaming to the ESP32!");
  Serial.println("Note: X-axis is perpendicular to the short edge of the MDF");
  Serial.println("=========================================");
}

// ==========================================
// AUTONOMOUS LOOP (Runs when browser is closed)
// ==========================================
void loop() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(100);
    return;
  }

  // Call our unified detection algorithm
  BlobResult blob = detectBlob(fb);

  // send Coords
  if (blob.found) {
    Serial1.printf("X:%d,Y:%d\n", blob.final_x, blob.final_y);
    Serial.printf("[Auto] Raw: %d,%d | Output -> X:%d, Y:%d\n", blob.raw_x, blob.raw_y, blob.final_x, blob.final_y);
  } else {
    Serial1.println("NONE");
  }

  // FREE MEMORY INSTANTLY
  esp_camera_fb_return(fb);

  // Delay prevents overwhelming the Main ESP32's Serial buffer
  delay(100); 
}