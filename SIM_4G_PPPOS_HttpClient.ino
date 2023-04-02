#include "Arduino.h"
#include "esp_camera.h"
#include "SPI.h"
#include "driver/rtc_io.h"
#include <FS.h>
#include <SPIFFS.h>
#include <PPPOS.h>
#include <PPPOSClient.h>
#include <PubSubClient.h>
#include <ArduinoHttpClient.h>

#define GSM_SERIAL          1
#define GSM_RX              2      // ESP32 TX - SIM RX 16  --- DO NOT USE PORT 4 because it is used by the camera
#define GSM_TX              14      // ESP32 RX - SIM TX 14
#define GSM_POWER           15      // GSM Power Enable 15
#define GSM_BR              115200  // Baudrate

const char* mqttServer = "test.mosquitto.org";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
const char* msgTopic = "esp32/dunv";
const char* callbackTopic = "dunv/esp32";

String serverName = "thongnhan.000webhostapp.com";   // Upload server address
String serverPath = "/upload.php";     // The default serverPath should be upload.php
const int serverPort = 80;

const int CHUNK_SIZE = 1024;  // Maximum LTE RX size is 1500 bytes
const int MAX_FILE_SIZE = 10000; // Maximum LTE Memory handle ??
const int POST_TIME_OUT = 10000;

PPPOSClient espClient;
PubSubClient mClient(espClient);
HttpClient hClient = HttpClient(espClient, serverName, serverPort);

#define TRIGGER_MODE // Photo capture triggered by GPIO pin rising/falling
// CONSTANTS
// GPIO Pin 33 is small red LED near to RESET button on the back of the board
const byte ledPin = GPIO_NUM_33;
// GPIO Pin 4 is bright white front-facing LED 
const byte flashPin = GPIO_NUM_4;
// When using TRIGGER_MODE, this pin will be used to initiate photo capture
const byte triggerPin = GPIO_NUM_13;
// Flash strength (0=Off, 255=Max Brightness)
// Setting a low flash value can provide a useful visual indicator of when a photo is being taken
const byte flashPower = 0;
#ifdef TIMED_MODE
  const int timeLapseInterval = 5; // seconds between successive shots in TIMELAPSE mode
#endif
const int startupDelayMillis = 3000; // time to wait after initialising  camera before taking photo

void sleep() {
  Serial.println("Start sleeping ...");
  // IMPORTANT - we define pin mode for the trigger pin at the end of setup, because most pins on the ESP32-CAM
  // have dual functions, and may have previously been used by the camera or SD card access. So we overwrite them here
  pinMode(triggerPin, INPUT_PULLDOWN);//DOWN
   //pinMode(triggerPin, INPUT_PULLUP);
  // Ensure the flash stays off while we sleep
  rtc_gpio_hold_en(GPIO_NUM_4);
  // Turn off the LED
  digitalWrite(ledPin, HIGH);
  delay(1000);
  #ifdef TRIGGER_MODE
    // Use this to wakeup when trigger pin goes HIGH
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 1);//1
    // Use this to wakeup when trigger pin goes LOW
    // esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 0);
  #elif defined(TIMED_MODE)
    // Or, use this to wakeup after a certain amount of time has elapsed (parameter specified in uS, so multiply secs by 1000000)
    esp_sleep_enable_timer_wakeup(timeLapseInterval * 1000000);
  #endif
  Serial.println("Going to sleep now");
  delay(100);
  esp_deep_sleep_start();
  delay(3000);
  Serial.println("This will never be printed");
}


#define CAMERA_MODEL_AI_THINKER
#if defined(CAMERA_MODEL_AI_THINKER)
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
#else
  #error "Camera model not selected"
#endif

// Photo File Name to save in SPIFFS
#define FILE_PHOTO "/photo.jpg"

String buffer = "";
char *data = (char *) malloc(1024); 
bool atMode = true;

bool sendCommandWithAnswer(String cmd, String ans){
   PPPOS_write((char *)cmd.c_str());
   unsigned long _tg = millis();
   while(true){
    data = PPPOS_read();
    if (data != NULL){
      char* command = strtok(data, "\n");
      while (command != 0)
      {
        buffer = String(command);
        buffer.replace("\r", "");
        command = strtok(0, "\n");
        if (buffer != "") { Serial.println(buffer); }
        if (buffer == ans) {buffer = ""; return true; }
        buffer = "";
      } 
    }
    if (millis() > (_tg + 5000)) { buffer = ""; return false; } 
   }
   buffer = "";
   return false;
}

int8_t  AT_CheckCSQ(void){
   int csq = 0;
   PPPOS_write("AT+CSQ\r\n");
   unsigned long _tg = millis();
   while(true){
    data = PPPOS_read();
    if (data != NULL){
      char* command = strtok(data, "\n");
      while (command != 0)
      {
        buffer = String(command);
        buffer.replace("\r", "");
        command = strtok(0, "\n");
        if (buffer != "") { Serial.println(buffer); }
        if(buffer.indexOf("+CSQ:") >= 0)
        {
          sscanf(buffer.c_str(),"+CSQ: %d", &csq);
          if(csq == 99) csq = 0;
          return csq;
        }
        buffer = "";
      } 
    }
    if (millis() > (_tg + 5000)) { buffer = ""; return false; } 
   }
   buffer = "";
   return csq;
}

bool startPPPOS(){  
  if (!sendCommandWithAnswer("ATD*99***1#\r\n", "CONNECT 115200")) { return false; }
  atMode = false;
  PPPOS_start(); 
  unsigned long _tg = millis();
  while(!PPPOS_isConnected()) {
    if (millis() > (_tg + 10000)) { PPPOS_stop();  atMode = true; return false; }
  }
  Serial.println("PPPOS Started");
  return true;
}

void SIM_reset(void)
{
  digitalWrite(GSM_POWER, LOW);
  delay(100);
  digitalWrite(GSM_POWER, HIGH);
}

void SIM_connect_PPP(void)
{
  while(true)
  {
      SIM_reset();
      bool SyncOK = false;

      Serial.println("SIM Sync"); 
      for(int i = 0; i < 500; i++)
      {
        delay(500);
        Serial.print("."); 
        if(!sendCommandWithAnswer("AT\r\n", "OK")){
          continue;
        }
       
      if(!sendCommandWithAnswer("AT+ CREG?\r\n", "OK")){
          continue;
        }
        
        SyncOK = true;
        break;
      }
      
      Serial.println("Check Signal Quality"); 
      for(int i = 0; i < 500; i++){
        if(AT_CheckCSQ() > 0){
          break;   
        }
        delay(1000);
      }

      for(int i = 0; i < 500; i++){
        data = PPPOS_read();
        if (data != NULL){
          Serial.println(data);  
        }
        delay(100);
      }

      if(SyncOK){

        Serial.println("Start PPPOS"); 
        if (startPPPOS()) { 
          Serial.println("Entering PPPOS... OK"); 
          break;
        } else { Serial.println("Entering PPPOS... Failed"); }
      }
  }
}


void initCamera()
{
   if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    mClient.publish(msgTopic, "An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
    mClient.publish(msgTopic, "SPIFFS mounted successfully");
  }
  
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
  
  if(psramFound()){ //check if pseudo ram is available
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
// Use PWM channel 7 to control the white on-board LED (flash) connected to GPIO 4
  ledcSetup(7, 5000, 8);
  ledcAttachPin(4, 7);
  // Turn the LED on at specified power
  ledcWrite(7, flashPower);

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    mClient.publish(msgTopic, "Camera init failed with error");
    return;
  }
  else
  {
    Serial.printf("Camera init successfully!");
    mClient.publish(msgTopic, "Camera init successfully!");
  }
  
// Turn flash off after taking picture
  ledcWrite(7, 0);
}

void setup()
{
  // Light up the discrete red LED on the back of the board to show the device is active
  pinMode(ledPin, OUTPUT);
  // It's an active low pin, so we write a LOW value to turn it on
  digitalWrite(ledPin, LOW);
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  
  Serial.begin(115200);
  Serial.println();

  /*  Init PPP  */
  PPPOS_init(GSM_TX, GSM_RX, GSM_BR, GSM_SERIAL, "", "");
  pinMode(GSM_POWER, OUTPUT);

  SIM_connect_PPP();
  
  mClient.setServer(mqttServer, mqttPort);
  mClient.setCallback(callback);

  mqttConnect();
  
  initCamera();

  delay(1000);
  sendPhoto();
  
  delay(1000);
  //sleep();
}

void loop() 
{ 
 if (!mClient.connected())
 {
   mqttConnect();       
 }
 else
 {
   mClient.loop();
 }
 
 delay(5000);
}

void sendPhoto(void) {  
  mClient.publish(msgTopic, "Start sending photo...");
  String getAll;
  String getBody;

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
  }
  
  Serial.println("Connecting to server: " + serverName);
  mClient.publish(msgTopic, "Connecting to server");
  
  if (hClient.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");
       
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint32_t imageLen = fb->len;
    uint32_t extraLen = head.length() + tail.length();
    uint32_t totalLen = imageLen + extraLen;

    Serial.println(String("Img Length: ") + imageLen);
  
    hClient.println("POST " + serverPath + " HTTP/1.1");
    hClient.println("Host: " + serverName);
    hClient.println("Content-Length: " + String(totalLen));
    hClient.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    hClient.println();
    hClient.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;   
    
    for (size_t n=0; n<fbLen; n=n+CHUNK_SIZE) {
      if (n+CHUNK_SIZE < fbLen) {
        hClient.write(fbBuf, CHUNK_SIZE);
        fbBuf += CHUNK_SIZE;
      }
      else if (fbLen%CHUNK_SIZE>0) {
        size_t remainder = fbLen%CHUNK_SIZE;
        hClient.write(fbBuf, remainder);
      }
    }   
    hClient.print(tail);
    
    esp_camera_fb_return(fb);
    
    int postTimeout = POST_TIME_OUT;
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + postTimeout) > millis()) {
      Serial.print(".");
      delay(100);      
      while (hClient.available()) {
        char c = hClient.read();
        if (c == '\n') {
          if (getAll.length()==0) { state=true; }
          getAll = "";
        }
        else if (c != '\r') { getAll += String(c); }
        if (state==true) { getBody += String(c); }
        startTimer = millis();
      }
      if (getBody.length()>0) { break; }
    }
    Serial.println();
    hClient.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connection to " + serverName +  " failed.";
    Serial.println(getBody);
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  Serial.println(messageTemp);
 
  sendPhoto();
}

void mqttConnect() {  
  while (!mClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mClient.connect("ESP32dunv")) {
      Serial.println("connected");
      // Subscribe
      mClient.subscribe(callbackTopic);
    } else {
      Serial.print("failed, rc=");
        Serial.print(mClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  } 
}
