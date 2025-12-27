#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <ESP32Servo.h>

// Replace with your network credentials
const char* ssid = "SSID";
const char* password = "PASSWORD";

// ESP32-CAM AI-Thinker Pin Definition
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// SIMPLE PIN CONFIGURATION - Just 2 pins alternating
#define PIN_A  13  // GPIO 13 - First pin
#define PIN_B  14  // GPIO 14 - Second pin

// SERVO CONFIGURATION
#define SERVO_PIN     12   // GPIO 12 for servo control
#define SERVO_MIN_US  500  // Minimum pulse width in microseconds
#define SERVO_MAX_US  2500 // Maximum pulse width in microseconds

// Timing configuration
#define SWITCH_INTERVAL_MS  10000  // 10 seconds for pin switching
#define SERVO_INTERVAL_MS   15000  // 15 seconds for servo rotation
#define SIGNAL_HIGH         HIGH

//FLASH - LED
#define FLASH_LED 4

// Global variables for pin control
unsigned long lastSwitchTime = 0;
bool pinAActive = true;  // true = PIN_A active, false = PIN_B active
bool pinControlEnabled = true;

// Global variables for servo control
Servo plantServo;
unsigned long lastServoTime = 0;
int currentServoAngle = 90;    // Current servo position (start at center)
bool servoEnabled = true;
bool servoDirection = true;    // true = moving to 180, false = moving to 0

WebServer server(80);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  
  // Initialize pins
  Serial.println("Initializing ESP32-CAM simple pin control...");
  initializePins();
  
  // Initialize servo
  Serial.println("Initializing servo control...");
  plantServo.attach(SERVO_PIN);
  
  // Initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  } else {
    Serial.println("Camera initialized successfully!");
  }
  
  // Get camera sensor and configure
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_quality(s, 9);
  s->set_brightness(s, 0);
  s->set_contrast(s, 1);
  s->set_saturation(s, 1);
  s->set_special_effect(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 0);
  s->set_ae_level(s, 0);
  s->set_gain_ctrl(s, 1);
  s->set_gainceiling(s, (gainceiling_t)0);
  s->set_bpc(s, 0);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  s->set_dcw(s, 1);
  s->set_colorbar(s, 0);
  
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("Camera Stream Ready! Connect to: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
  
  // Start web server
  server.on("/", handleRoot);
  server.on("/stream", handleStream);
  server.on("/capture", handleCapture);
  server.on("/status", handleStatus);
  server.on("/pins/enable", handlePinEnable);
  server.on("/pins/disable", handlePinDisable);
  server.on("/pins/test", handlePinTest);
  server.begin();
  
  lastSwitchTime = millis();
  lastServoTime = millis();
  Serial.println("Plant monitoring system ready!");
  Serial.println("Pin switching: GPIO 13 <-> GPIO 14 every 10 seconds");
  Serial.println("Servo control: GPIO 12, 15-second intervals");

  pinMode(FLASH_LED, OUTPUT);
}

void loop() {
  server.handleClient();
  handlePinSequence();
  // Move from 0° to 180°
  for (int pos = 0; pos <= 180; pos++) {
    plantServo.write(pos);
    delay(15);  // Adjust speed
  }

  digitalWrite(FLASH_LED, HIGH);
  delay(5000);
  digitalWrite(FLASH_LED, LOW);


  // Move from 180° back to 0°
  for (int pos = 180; pos >= 0; pos--) {
    plantServo.write(pos);
    delay(15);
  }

  digitalWrite(FLASH_LED, HIGH);
  delay(5000);
  digitalWrite(FLASH_LED, LOW);

}

void initializePins() {
  // Initialize the two switching pins
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  
  // Start with PIN_A active, PIN_B inactive
  digitalWrite(PIN_A, SIGNAL_HIGH);
  digitalWrite(PIN_B, LOW);
  
  Serial.printf("GPIO %d (PIN_A): Initialized and active\n", PIN_A);
  Serial.printf("GPIO %d (PIN_B): Initialized and inactive\n", PIN_B);
  Serial.println("Simple 2-pin control ready!");
}

void handlePinSequence() {
  if (!pinControlEnabled) return;
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastSwitchTime >= SWITCH_INTERVAL_MS) {
    // Switch active pin
    pinAActive = !pinAActive;
    lastSwitchTime = currentTime;
    
    if (pinAActive) {
      // Activate PIN_A, deactivate PIN_B
      digitalWrite(PIN_A, SIGNAL_HIGH);
      digitalWrite(PIN_B, LOW);
      Serial.printf("Switched to PIN_A (GPIO %d)\n", PIN_A);
    } else {
      // Activate PIN_B, deactivate PIN_A
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, SIGNAL_HIGH);
      Serial.printf("Switched to PIN_B (GPIO %d)\n", PIN_B);
    }
  }
}


void handleRoot() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "<title>ESP32-CAM Simple Plant Monitor</title></head>";
  html += "<body style=\"font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px;\">";
  html += "<h1>ESP32-CAM Simple Plant Monitor</h1>";
  html += "<p><img src=\"/stream\" style=\"width:100%;max-width:800px\" /></p>";
  
  html += "<h2>Camera Controls</h2>";
  html += "<p>";
  html += "<a href=\"/capture\"><button style=\"background-color: #4CAF50; border: none; color: white; padding: 15px 32px; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer;\">Capture Photo</button></a>";
  html += "<a href=\"/status\"><button style=\"background-color: #008CBA; border: none; color: white; padding: 15px 32px; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer;\">Status</button></a>";
  html += "</p>";
  
  html += "<h2>Pin Control</h2>";
  html += "<p>Active Pin: <strong>GPIO " + String(pinAActive ? PIN_A : PIN_B) + "</strong></p>";
  html += "<p>Status: <strong>" + String(pinControlEnabled ? "Enabled" : "Disabled") + "</strong></p>";
  html += "<p>";
  html += "<a href=\"/pins/enable\"><button style=\"background-color: #4CAF50; border: none; color: white; padding: 10px 20px; font-size: 14px; margin: 2px; cursor: pointer;\">Enable Pins</button></a>";
  html += "<a href=\"/pins/disable\"><button style=\"background-color: #f44336; border: none; color: white; padding: 10px 20px; font-size: 14px; margin: 2px; cursor: pointer;\">Disable Pins</button></a>";
  html += "<a href=\"/pins/test\"><button style=\"background-color: #ff9800; border: none; color: white; padding: 10px 20px; font-size: 14px; margin: 2px; cursor: pointer;\">Test Pins</button></a>";
  html += "</p>";
  
  html += "<h2>Servo Control</h2>";
  html += "<p>Current Position: <strong>" + String(currentServoAngle) + "°</strong></p>";
  html += "<p>Status: <strong>" + String(servoEnabled ? "Auto-Rotating" : "Disabled") + "</strong></p>";
  
  html += "<h3>System Configuration:</h3>";
  html += "<p>Pin Switching: GPIO 13 <-> GPIO 14 (10s interval)<br>";
  html += "Servo: GPIO 12 (15s interval, 0° <-> 180°)</p>";
  
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleStream() {
  WiFiClient client = server.client();
  
  if (!client.connected()) {
    Serial.println("Client not connected for streaming");
    return;
  }
  
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);
  
  while (client.connected()) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      break;
    }
    
    if (fb->len > 100) {
      String head = "--frame\r\n";
      head += "Content-Type: image/jpeg\r\n";
      head += "Content-Length: " + String(fb->len) + "\r\n\r\n";
      
      server.sendContent(head);
      client.write((char*)fb->buf, fb->len);
      server.sendContent("\r\n");
      
      esp_camera_fb_return(fb);
      delay(66);
    } else {
      Serial.println("Frame too small, skipping");
      esp_camera_fb_return(fb);
      delay(100);
    }
    
    yield();
  }
  
  Serial.println("Client disconnected from stream");
}

void handleCapture() {
  camera_fb_t * fb = esp_camera_fb_get();
  
  if (!fb) {
    Serial.println("Camera capture failed");
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }
  
  Serial.printf("Captured frame size: %d bytes\n", fb->len);
  
  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  
  esp_camera_fb_return(fb);
}

void handleStatus() {
  String json = "{";
  json += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"uptime\":" + String(millis()) + ",";
  json += "\"camera_initialized\":" + String(esp_camera_sensor_get() ? "true" : "false") + ",";
  json += "\"pin_control_enabled\":" + String(pinControlEnabled ? "true" : "false") + ",";
  json += "\"active_pin\":" + String(pinAActive ? PIN_A : PIN_B) + ",";
  json += "\"pin_a\":" + String(PIN_A) + ",";
  json += "\"pin_b\":" + String(PIN_B) + ",";
  json += "\"servo_enabled\":" + String(servoEnabled ? "true" : "false") + ",";
  json += "\"servo_position\":" + String(currentServoAngle) + ",";
  json += "\"servo_pin\":" + String(SERVO_PIN);
  json += "}";
  
  server.send(200, "application/json", json);
}

void handlePinEnable() {
  pinControlEnabled = true;
  lastSwitchTime = millis();
  Serial.println("Pin control enabled");
  server.send(200, "text/plain", "Pin control enabled");
}

void handlePinDisable() {
  pinControlEnabled = false;
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  Serial.println("Pin control disabled - both pins OFF");
  server.send(200, "text/plain", "Pin control disabled");
}

void handlePinTest() {
  Serial.println("=== PIN TEST SEQUENCE ===");
  
  // Test PIN_A
  digitalWrite(PIN_A, HIGH);
  digitalWrite(PIN_B, LOW);
  delay(2000);
  Serial.printf("GPIO %d (PIN_A): ON\n", PIN_A);
  
  // Test PIN_B
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, HIGH);
  delay(2000);
  Serial.printf("GPIO %d (PIN_B): ON\n", PIN_B);
  
  // Reset to normal state
  if (pinAActive) {
    digitalWrite(PIN_A, SIGNAL_HIGH);
    digitalWrite(PIN_B, LOW);
  } else {
    digitalWrite(PIN_A, LOW);
    digitalWrite(PIN_B, SIGNAL_HIGH);
  }
  
  Serial.println("=== PIN TEST COMPLETE ===");
  server.send(200, "text/plain", "Pin test complete - check Serial Monitor");
}
