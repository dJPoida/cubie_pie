#define projectName "ESP32 Async WebSocket Test"
#define projectVersion "v1.00"

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//https://github.com/me-no-dev/ESPAsyncWebServer

//Control Messages
#define MSG_SET_NECK_POS 1100
#define MSG_SET_PAN_POS 1110

//Response Messages
#define MSG_HEARTBEAT_RESPONSE 9000

//Servo Numbers
#define SERVO_NUM_NECK 0
#define SERVO_NUM_TILT 1
#define SERVO_NUM_PAN 2

//Neck Elevation
#define SERVO_NECK_MIN  230 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_NECK_MAX  400 // this is the 'maximum' pulse length count (out of 4096)

//Head Elevation
#define SERVO_TILT_MIN  240 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_TILT_MAX  400 // this is the 'maximum' pulse length count (out of 4096)

//Head Pan
#define SERVO_PAN_MIN  180 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_PAN_MAX  600 // this is the 'maximum' pulse length count (out of 4096)



const char* ssid = "ELDROIDZ";
const char* password = "pkmj4eva";

unsigned int elevationDurationMS = 750;
unsigned int defaultElevationDurationMS = 750;
unsigned int panDurationMS = 1000;
unsigned int defaultPanDurationMS = 1000;

unsigned long startNeckMovementMillis;
float startNeckServoPWM = 0;
float currentNeckServoPWM = SERVO_NECK_MIN - 1;
float targetNeckServoPWM = 0;

unsigned long startTiltMovementMillis;
float startTiltServoPWM = 0;
float currentTiltServoPWM = SERVO_TILT_MIN - 1;
float targetTiltServoPWM = 0;

unsigned long startPanMovementMillis;
float startPanServoPWM = 0;
float currentPanServoPWM = SERVO_PAN_MIN + ((SERVO_PAN_MAX - SERVO_PAN_MIN) / 2) - 1; //Start somewhere close to 50%
float targetPanServoPWM = 0;

int servoUpdateInterval = 5000; //Microseconds

//Globals
AsyncWebServer server(80);
AsyncWebSocket ws("/");
Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40);

//ESP32 Timer
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;
void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void initTimer() {
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(1, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function (value in microseconds) and repeat the alarm.
  timerAlarmWrite(timer, servoUpdateInterval, true);

  // Start an alarm
  timerAlarmEnable(timer);
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    Serial.printf("Websocket Client [%u] Connected\n", client->id());
    sendWSMessage(MSG_HEARTBEAT_RESPONSE, String(client->id()));
    client->ping();
  } else if(type == WS_EVT_DISCONNECT){
    Serial.printf("Websocket Client [%u] Disconnected\n", client->id());
  } else if(type == WS_EVT_ERROR){
    Serial.printf("Websocket Error (Client [%u]): %u %s\n", client->id(), *((uint16_t*)arg), (char*)data);
  } else if(type == WS_EVT_DATA){
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    String msg = "";

    //the whole message is in a single frame and we got all of it's data
    if(info->final && info->index == 0 && info->len == len){
      //Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);

      if(info->opcode == WS_TEXT){
        for(size_t i=0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      Serial.printf("%s\n",msg.c_str());
      handleWSMessage(msg.c_str());
    } 
    
    //message is comprised of multiple frames or the frame is split into multiple packets
    else {
      if(info->opcode == WS_TEXT){
        for(size_t i=0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }

      if((info->index + len) == info->len){
        if(info->final){
          handleWSMessage(msg.c_str());
        }
      }
    }
  }
}



void setup(){
  Serial.begin(115200);
  Serial.println(projectName);
  Serial.println(projectVersion);
  Serial.println("Initialising....");

  Serial.println(" ");
  Serial.println(" ----------");
  Serial.print(" - Servo Controller...");
  servoDriver.begin();
  servoDriver.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  //Connect to the WiFi
  Serial.println(" ");
  Serial.println(" ----------");
  Serial.print(" - WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.print(" Retrying...");
    WiFi.disconnect(false);
    delay(1000);
    WiFi.begin(ssid, password);
  }
  Serial.print(" Connected! IP address: ");
  Serial.println(WiFi.localIP());
  
  Serial.println(" ");
  Serial.println(" ----------");
  Serial.print(" - Web Socket...");
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  Serial.println(" ");
  Serial.println(" ----------");
  Serial.print(" - Initial Positioning...");
  setHeadPanPos(50, 1);
  setNeckPos(0, 1);
 
  Serial.println(" ");
  Serial.println(" ----------");
  Serial.print(" - Timers...");
  initTimer();

  Serial.println(" ");
  Serial.println(" Initialised! ");
  Serial.println(" ");

  delay(100);
}

void loop(){
  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);

    updateServos();
  }
}



void handleWSMessage(String message) {
  unsigned int code = message.substring(0, 4).toInt();
  String data = message.substring(5);
  
  Serial.print("WSRecv: ");
  Serial.printf("%d %s\n", code, data.c_str());

  switch (code) {
    case MSG_SET_NECK_POS:
      setNeckPos(data.toInt(), 0);
      break;
    case MSG_SET_PAN_POS:
      setHeadPanPos(data.toInt(), 0);
      break;
  }
  
  return;
}



void sendWSMessage(const unsigned int code, String data) {
  Serial.print("WSSend: ");
  Serial.printf("%d %s\n", code, data.c_str());
  ws.printfAll("%d %s", code, data.c_str());
}


void setNeckPos(byte pos, int duration) {
  //Use the default duration
  if (duration == 0) {
    elevationDurationMS = defaultElevationDurationMS;
  } 
  
  //Use ths specified duration
  else {
    elevationDurationMS = duration;
  }

  startNeckServoPWM = constrain(currentNeckServoPWM, SERVO_NECK_MIN, SERVO_NECK_MAX);
  targetNeckServoPWM = map(pos, 0, 100, SERVO_NECK_MIN, SERVO_NECK_MAX);
  startNeckMovementMillis = millis();

  startTiltServoPWM = constrain(currentTiltServoPWM, SERVO_TILT_MIN, SERVO_TILT_MAX);
  targetTiltServoPWM = map(pos, 0, 100, SERVO_TILT_MIN, SERVO_TILT_MAX);
  startTiltMovementMillis = millis();

  Serial.printf("Moving Neck: StartPWM=%.2f targetPWM=%.2f startMillis=%d duration=%d\n", startNeckServoPWM, targetNeckServoPWM, startNeckMovementMillis, elevationDurationMS);
  Serial.printf("Moving Tilt: StartPWM=%.2f targetPWM=%.2f startMillis=%d duration=%d\n", startTiltServoPWM, targetTiltServoPWM, startTiltMovementMillis, elevationDurationMS);
}


void setHeadPanPos(byte pos, int duration) {
  //Use the default duration
  if (duration == 0) {
    panDurationMS = defaultPanDurationMS;
  } 
  
  //Use ths specified duration
  else {
    panDurationMS = duration;
  }

  startPanServoPWM = constrain(currentPanServoPWM, SERVO_PAN_MIN, SERVO_PAN_MAX);
  targetPanServoPWM = map(pos, 100, 0, SERVO_PAN_MIN, SERVO_PAN_MAX); //INVERTED TO MAKE 0 = LEFT
  startPanMovementMillis = millis();

  Serial.printf("Moving Pan: StartPWM=%.2f targetPWM=%.2f startMillis=%d duration=%d\n", startPanServoPWM, targetPanServoPWM, startPanMovementMillis, panDurationMS);
}


void updateServos() {
  unsigned long currentTime = millis();
  
  //Do we need to move the neck?
  if (targetNeckServoPWM != currentNeckServoPWM) {
    currentNeckServoPWM = updateServo(SERVO_NUM_NECK, currentNeckServoPWM, targetNeckServoPWM, startNeckServoPWM, startNeckMovementMillis, elevationDurationMS);
  }

  //Do we need to move the tilt?
  if (targetTiltServoPWM != currentTiltServoPWM) {
    currentTiltServoPWM = updateServo(SERVO_NUM_TILT, currentTiltServoPWM, targetTiltServoPWM, startTiltServoPWM, startTiltMovementMillis, elevationDurationMS);
  }
  
  //Do we need to move the pan?
  if (targetPanServoPWM != currentPanServoPWM) {
    currentPanServoPWM = updateServo(SERVO_NUM_PAN, currentPanServoPWM, targetPanServoPWM, startPanServoPWM, startPanMovementMillis, elevationDurationMS);
  }
}


float updateServo (int servoNum, float currentServoPWM, float targetServoPWM, float startServoPWM, unsigned long startMovementMillis, unsigned long movementDuration) {
  float returnServoPWM = currentServoPWM;
  
  //Do we need to move the neck?
  if (targetServoPWM != currentServoPWM) {
    float newServoPWM = easeInOutCubic((millis() - startMovementMillis), startServoPWM, (targetServoPWM - startServoPWM), movementDuration);

    //Going Up? (include a +1 to ignore fractional curves within a single PWM increment)
    if (targetServoPWM > startServoPWM) {
      if ((newServoPWM + 1) > targetServoPWM) {
        newServoPWM = targetServoPWM;
      }
    } else {
      if ((newServoPWM - 1) < targetServoPWM) {
        newServoPWM = targetServoPWM;
      }
    }

    if (newServoPWM != currentServoPWM) {
      returnServoPWM = newServoPWM;
      Serial.printf("Servo %d PWM: %.2f\n", servoNum, currentNeckServoPWM);
      
      //set the neck
      servoDriver.setPWM(servoNum, 0, int(returnServoPWM));
    }
  }

  return returnServoPWM;
}


/**
 * cubic easing in/out - acceleration until halfway, then deceleration
 * t = startTime (ms)
 * b = baseValue
 * c = changeInValue
 * d = duration (ms)
 */
float easeInOutCubic (float t, float b, float c, float d) {
  if ((t/=d/2) < 1) return c/2*t*t*t + b;
  return c/2*((t-=2)*t*t + 2) + b;
}

