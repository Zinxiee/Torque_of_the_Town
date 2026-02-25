#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ==========================================
// 1. WIFI NETWORK
// ==========================================
const char* ssid = "RobotVision"; 
const char* password = "TorqueOfTheTown";

// ==========================================
// 2. BLOB DETECTION SETTINGS
// ==========================================
const int WHITE_THRESHOLD = 195; 
const int MIN_PIXELS = 50;

// toggle debug mode to see binary mask (black and white thresholding)
const bool DEBUG_MASK_MODE = false;

// ==========================================
// 3. CROP BOX (REGION OF INTEREST)
// ==========================================
// Adjust these to cut out the robot/background. 
// Max Width is 320, Max Height is 240.
const int CROP_X_MIN = 40;  // Ignore the left 40 pixels
const int CROP_X_MAX = 280; // Ignore the right 40 pixels
const int CROP_Y_MIN = 20;  // Ignore the top 20 pixels
const int CROP_Y_MAX = 240; // Ignore the bottom 0 pixels

// ==========================================
// PINOUT
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

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK) return res;

  // THIS LOOP RUNS CONTINUOUSLY (Continuous Tracking of disc)
  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    // finding centroid
    long sum_x = 0;
    long sum_y = 0;
    int white_pixel_count = 0;

    // --- 1. CENTROID MATH (within boundry box) ---
    // Instead of starting at 0, we start at CROP_MIN.
    for (int y = CROP_Y_MIN; y < CROP_Y_MAX; y++) {
      for (int x = CROP_X_MIN; x < CROP_X_MAX; x++) {
        int index = (y * fb->width) + x;
        uint8_t brightness = fb->buf[index];

        if (brightness > WHITE_THRESHOLD) {
          sum_x += x;
          sum_y += y;
          white_pixel_count++;

        // when debug mode is toggled:
        if (DEBUG_MASK_MODE) {
            fb->buf[index] = 255; // Force the pixel to Pure White on the stream
          }
        } else {
          if (DEBUG_MASK_MODE) {
            fb->buf[index] = 0; // Force the pixel to Pure Black on the stream
          }
        }
      }
    }

    // --- DRAW THE CROP BOX ON THE STREAM (For visuals) ---
    // Top and Bottom lines
    for(int i = CROP_X_MIN; i < CROP_X_MAX; i++) {
        fb->buf[CROP_Y_MIN * fb->width + i] = 100; // Gray line
        fb->buf[(CROP_Y_MAX-1) * fb->width + i] = 100;
    }
    // Left and Right lines
    for(int i = CROP_Y_MIN; i < CROP_Y_MAX; i++) {
        fb->buf[i * fb->width + CROP_X_MIN] = 100;
        fb->buf[i * fb->width + (CROP_X_MAX-1)] = 100;
    }

    // --- 2. DRAW CROSSHAIR ---
    if (white_pixel_count > MIN_PIXELS) {
      int cy = sum_x / white_pixel_count;
      int cx = sum_y / white_pixel_count;
      
      // Draw crosshair at the RAW optical location
      for(int i=-10; i<=10; i++){
        if(cy+i>=0 && cy+i<fb->width) fb->buf[cx*fb->width + (cy+i)] = 0; 
        if(cx+i>=0 && cx+i<fb->height) fb->buf[(cx+i)*fb->width + cy] = 0; 
      }

      // --- 3. CONTINUOUS STREAMING OVER WIRES ---
      // This prints constantly, effectively tracking the disc in real-time
      Serial1.printf("X:%d,Y:%d\n", cx, cy);
      Serial.printf("Disc at X:%d, Y:%d\n", cx, cy);
      
    } else {
      Serial1.println("NONE");
    }

    // COMPRESS & STREAM OVER WIFI
    bool jpeg_converted = frame2jpg(fb, 20, &_jpg_buf, &_jpg_buf_len); 
    esp_camera_fb_return(fb); 
    fb = NULL;

    if(!jpeg_converted){
      res = ESP_FAIL;
    }

    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    
    if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    } 
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

void setup() {
  Serial.begin(115200);
  
  // Start UART connection to Main ESP32 (D7=RX, D6=TX)
  Serial1.begin(9600, SERIAL_8N1, D7, D6);

  // disable this otherwise camera halts and waits for serial monitor:
  // while(!Serial); // DEBUG ONLY

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
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

  // software flipping of camera
  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 0);

  // --- CREATE THE WIFI NETWORK ---
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  startCameraServer();

  Serial.println("=========================================");
  Serial.println("HEYO CAMERA IS OPERATIONAL, TRACKING IS LIVE");
  Serial.print("Network Created! Connect to: ");
  Serial.println(ssid);
  Serial.print("Stream Video at: http://");
  Serial.println(IP);
  Serial.println("Coordinates are streaming to the esp32!");
  Serial.println("Note: X-axis is perpendicular to the short edge of the MDF");
  Serial.println("=========================================");
}

void loop() {
  delay(10000); 
}