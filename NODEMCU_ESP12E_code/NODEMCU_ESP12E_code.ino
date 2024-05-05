// Laboratory Oven control system - Version 4
// - MQTT - Mosquitto + Node REd + Influxdb2
// - PID control

#include <Arduino.h> 
#include "C:\Users\pbrug\Documents\01. PEB Personal\Credenciais Cloud\SecurityGit\credentials.h"

#include "EspMQTTClient.h"
#include <ESP8266mDNS.h> 

#include <ArduinoJson.h>
#include "DHTesp.h"
#include <PID_v1.h>

// DISABLE DEBUG FOR PRODUCTION
//#define DEBUG_DISABLED true

// SELECT BETWEEN SERIAL or NETWORK DEBUG
//#define REMOTE_DEBUG

#ifdef REMOTE_DEBUG
  // OVER NETWORK DEBUG
  #include "RemoteDebug.h"            //https://github.com/JoaoLopesF/RemoteDebug
  #define REMOTE_INITIAL_LEVEL 1      //(VERBOSE)
  #define TELNET_PORT 23
  #define HOST_NAME "remotedebug"
  #define USE_ARDUINO_OTA true
  RemoteDebug Debug;
#else
  // SERIAL DEBUGGER
  #include "SerialDebug.h"           // https://github.com/JoaoLopesF/SerialDebug
  #define DEBUG_INITIAL_LEVEL DEBUG_LEVEL_VERBOSE
  //#define DEBUG_DISABLE_DEBUGGER true
  //#define DEBUG_AUTO_FUNC_DISABLED true
  //#define DEBUG_USE_FLASH_F true   // IMPORTANT FOR ARDUINO - LOW MEMORY
  //#define DEBUG_MINIMUM true       // IMPORTANT FOR ARDUINO - LOW MEMORY    
#endif

// Configuring INFLUXDB

// Declare feeds to publish sensor data 
EspMQTTClient client(
  wifi_ssid,
  wifi_password,
  "192.168.27.4",  // MQTT Broker server ip
  MQTT_USERNAME,   // Can be omitted if not needed
  MQTT_PASSWORD,   // Can be omitted if not needed
  "LABOVEN",       // Client name that uniquely identify your device
  1883             // The MQTT port, default to 1883. this line can be omitted
);

// mDNS
const char* mdnsName = "LabOven";    // Domain name for the mDNS responder

// Temperature Sensors
#define DHTpin1 14   //D5 of NodeMCU is GPIO14
#define DHTpin2 12   //D6 of NodeMCU is GPIO12
DHTesp dhtA;
DHTesp dhtB;

// L298N Driver
int ENA = D7;
int IN1 = D1;
int IN2 = D2;

int ENB = D8;
int IN3 = D3;
int IN4 = D4;

// variables do retrieve and parse data from Mosquitto messages
// for some unknown (yet) reason, the receive string has maximum length=52
String payload_base = "";
String payload = "";
String json_response;
DynamicJsonDocument doc(1024);

// Declare variables that will be published to Google Sheets
int value0 = 0;
float value1 = 0;
float value2 = 0;
float value3 = 0;
float value4 = 0;
float value5 = 0;
float value6 = 0;
char value7[] = "..";
uint value8;

// Declare reading variables
double humidityA = 0;
double temperatureA = 0;
double humidityB = 0;
double temperatureB = 0;
int minsampleperiod = 2000;     // default min sample period for DHT22 in ms

TempAndHumidity dhtValuesA;
TempAndHumidity dhtValuesB;

// delay control variables
float elapsedTime, currentTime, previousTime;

// controlling parameters
double powerA = 0;
double powerB = 0;
double targetTempA = 30;
double targetTempB = 30;
int max_duty_cycleA = 127;  // =50% ->  [12V (power supply) - 2V (L298N voltage drop)] * max_duty_cycle = 5V
int max_duty_cycleB = 127;  // =50% ->  [12V (power supply) - 2V (L298N voltage drop)] * max_duty_cycle = 5V
int pwmA, pwmB;
double oldTempA, oldTempB;
int counter = 0;
double time_sec;
int time_delay = 15;
double Kp_A = 1, Kp_B = 1, new_Kp;
double Kd_A = 0, Kd_B = 0, new_Kd;
double Ki_A = 0, Ki_B = 0, new_Ki;
bool flagChangeKA = false;
bool flagChangeKB = false;
bool flagChangeDutyA = false;
bool flagChangeDutyB = false;
const int sampleRate = 2000;      // Time interval of the PID control in millisenconds
double debugcounter = 0;          // PEB TO BE DELETED

PID myPIDA (&temperatureA, &powerA, &targetTempA, Kp_A, Ki_A, Kd_A, DIRECT);
PID myPIDB (&temperatureB, &powerB, &targetTempB, Kp_B, Ki_B, Kd_B, DIRECT);

// ------------------------------------------
// SETUP FUNCTION
// ------------------------------------------
void setup() {
  // Serial Communication
  Serial.begin(115200);        
   while(! Serial);
   
  // Optional functionalities of EspMQTTClient
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater();    // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overridded with enableHTTPWebUpdater("user", "password").
  client.enableOTA(OTA_password, 8266);   // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).
  client.enableLastWillMessage("TestClient/lastwill", "I am going offline");  // You can activate the retain flag by setting the third parameter to true

	// Initialize RemoteDebug
  #ifdef REMOTE_DEBUG  
    Debug.begin(HOST_NAME, TELNET_PORT, REMOTE_INITIAL_LEVEL); // Initialize the WiFi server
    Debug.setResetCmdEnabled(true); // Enable the reset command
    Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
    Debug.showColors(true); // Colors
  #endif

  // Start Logging
  debugA("\n \n Booting MCU - v1.6 ...\n \n");

  // Onboard LED
  pinMode(LED_BUILTIN, OUTPUT);  // GPIO 16

  // Set Sensor A and B
  debugV("Setting up sensors...");
  dhtA.setup(DHTpin1, DHTesp::DHT22); //for DHT11 Connect DHT sensor to GPIO 14
  dhtB.setup(DHTpin2, DHTesp::DHT22); //for DHT11 Connect DHT sensor to GPIO 12
  minsampleperiod = max(dhtA.getMinimumSamplingPeriod(), dhtB.getMinimumSamplingPeriod());

  // Setup L298N driver pins
  debugV("Setting up L298N pins...");
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT); 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT); 
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  previousTime = millis(); 

  // Setting up PID controllers
  myPIDA.SetMode(AUTOMATIC);
  myPIDB.SetMode(AUTOMATIC);
  myPIDA.SetSampleTime(sampleRate); // Assign the sample rate of the control
  myPIDB.SetSampleTime(sampleRate); // Assign the sample rate of the control
  myPIDA.SetOutputLimits(0.0, 1.0);
  myPIDB.SetOutputLimits(0.0, 1.0);
  update_PID_settings();

  // // Start mDNS
  startmDNS(); 
}

// ------------------------------------------
// MAIN LOOP
// ------------------------------------------
void loop() {

  // Debug OMM
  debugV("*** Begin Loop - Free Heap Memory:%u\t Max Block Size:%u\n", ESP.getFreeHeap(), ESP.getMaxFreeBlockSize());

  // After each time_delay log retrieve parameters from google sheet and write log
  currentTime = millis();                           // get current time
  elapsedTime = (currentTime - previousTime)/1000;  // in seconds

  if (elapsedTime > time_delay) {
    debugcounter += time_delay/60.0;

    publish_data_to_mqtt();

    previousTime = currentTime;      // the last "trigger time" is stored 
  }

  // Perform controlling activities - Get data from sensors and update heater power
  get_data_from_sensors();
  set_heater_power_with_PID();
  
  // MDNS handle
  MDNS.update(); 

  // RemoteDebug handle
  debugHandle();

  // wifi & MQTT client stuff
  client.loop();

  // a delay due to DHT sensor limitations
  delay(minsampleperiod);
}

// -------------------------------------------------------------------
// MAIN LABORATORY OVEN FUNCTIONS (to be replace for a class+methods)
// -------------------------------------------------------------------
void get_data_from_sensors(){
  
  debugI("Retrieve data from sensor...");

  // store previous temperatures
  oldTempA = temperatureA;
  oldTempB = temperatureB;

  // Retrieve data from sensor A
  //sensorError = "...";
  counter = 0;
  
  // time in seconds when sensor measures were taken
  time_sec = millis()/1000;

  while (true) {
    dhtValuesA = dhtA.getTempAndHumidity();
    if (dhtA.getStatus() == 0){
      humidityA = dhtValuesA.humidity;
      temperatureA = dhtValuesA.temperature - 1;
      break;
    }
    if (counter++ == 3){
      // sensorError = sensorError + " ErrorA";// + dhtA.getStatusString() + "\t";
      value7[0] = 'A';
      break;
    }
    delay(dhtA.getMinimumSamplingPeriod());
  };

  // Retrieve data from sensor B
  counter = 0;
  while (true) {
    dhtValuesB = dhtB.getTempAndHumidity();
    if (dhtB.getStatus() == 0){
      humidityB = dhtValuesB.humidity;
      temperatureB = dhtValuesB.temperature;
      break;
    }
    if (counter++ == 3){
      //sensorError = sensorError + " ErrorB"; // + dhtB.getStatusString() + "\t";
      value7[1] = 'B';
      break;
    }
    delay(dhtB.getMinimumSamplingPeriod());
  }

  // print status
  debugV(" - Sensor A >> Status:%s\tHumidity:%f\tTemperature:%f",  dhtA.getStatusString(), humidityA, temperatureA);
  debugV(" - Sensor B >> Status:%s\tHumidity:%f\tTemperature:%f\n",  dhtB.getStatusString(), humidityB, temperatureB);
}

void update_PID_settings(){

  debugI("Updating PID settings if necessary...");  

  if (flagChangeKA) {
    myPIDA.SetTunings(Kp_A, Ki_A, Kd_A);
    debugV("PDI parameters for A were updated to (Kp, Ki, Kd) = (%f, %f, %f)", Kp_A, Ki_A, Kd_A);
  }
  if (flagChangeKB) {
    myPIDB.SetTunings(Kp_B, Ki_B, Kd_B);
    debugV("PDI parameters for B were updated to (Kp, Ki, Kd) = (%f, %f, %f)", Kp_B, Ki_B, Kd_B);
  }
  if (flagChangeKA || flagChangeKB) {
  debugV("\n");
  }  
}

void set_heater_power_with_PID(){

  debugI("Setting PWM signals for heater element power...");

  // compute powerA and powerB control signal (0 to 100%)
  myPIDA.Compute();
  myPIDB.Compute();

  // convert scale [0, 100%] into [0, max_duty_cycle] and set PWM signal
  pwmA = (int) round(powerA * max_duty_cycleA);
  pwmB = (int) round(powerB * max_duty_cycleB);

  // set PWM values
  analogWrite(ENA, pwmA);
  analogWrite(ENB, pwmB);

  // print status
  debugV(" - Power >> Heater A:%3.1f\tHeater B:%3.1f", 100*powerA, 100*powerB);
  debugV(" - PWM   >> Heater A:%d\tHeater B:%d\n", pwmA, pwmB);
}

void publish_data_to_mqtt(){
  // Log activity
  debugI("Publishing to Mosquitto\n");

  // prepare payload and publish
  payload = payload_base + "{\"s\": " + "\"Sensor A\"" + ", \"t\": " + temperatureA + ", \"h\": " + humidityA + ", \"p\": " + (powerA*100.0) + ", \"tt\": " + targetTempA + "}";
  client.publish("/laboven/sensorA", payload);

  payload = payload_base + "{\"s\": " + "\"Sensor B\"" + ", \"t\": " + temperatureB + ", \"h\": " + humidityB + ", \"p\": " + (powerB*100.0) + ", \"tt\": " + targetTempB + "}";
  client.publish("/laboven/sensorB", payload);

  client.publish("/laboven/counter", String(debugcounter)); 
}

#include <cstdlib> 
float parseFloat(const String &str) {
    // Use atof to convert string to float
    return atof(str.c_str());
}

void handleMessage_delay(const String & payload) {
  debugI("Receiving published feed... - Delay");
  time_delay = (int) parseFloat(payload);
  debugV(" - New delay value: %u", time_delay);
}

void handleMessage_configA(const String & payload) {
  debugI("Receiving published feed... - Config A");
  json_response = payload;
  deserializeJson(doc, json_response);    
  debugV(" - json >> %u", json_response.length());  
  debugV(" - Kd >> %f", doc["Kd"]);

  targetTempA = doc["t"];
  max_duty_cycleA = (int) doc["m"];

  if ( Kp_A != doc["p"] ||  Kd_A != doc["d"] || Ki_A != doc["i"]){
    flagChangeKA = true;
    Kp_A = doc["p"];
    Kd_A = doc["d"];
    Ki_A = doc["i"];
  } else {
    flagChangeKA = false;
  }  

  debugV(" - Data A >> Target: %f\tDuty: %d\t(Kp, Ki, Kd):(%f , %f, %f)\n", targetTempA, max_duty_cycleA, Kp_A, Ki_A, Kd_A);

  update_PID_settings();
}

void handleMessage_configB(const String & payload) {
  debugI("Receiving published feed... - Config B");
  json_response = payload;
  deserializeJson(doc, json_response);    

  targetTempB = doc["t"];
  max_duty_cycleB = (int) doc["m"];

  if ( Kp_B != doc["p"] ||  Kd_B != doc["d"] || Ki_B != doc["i"]){
    flagChangeKB = true;
    Kp_B = doc["p"];
    Kd_B = doc["d"];
    Ki_B = doc["i"];
  } else {
    flagChangeKB = false;
  }

  debugV(" - Data B >> Target: %f\tDuty: %d\t(Kp, Ki, Kd):(%f , %f, %f)\n", targetTempB, max_duty_cycleB, Kp_B, Ki_B, Kd_B);

  update_PID_settings();
}

void onConnectionEstablished()
{
  // Subscribe to "mytopic/test" and display received message to Serial
  client.subscribe("/laboven/delay", handleMessage_delay);
  client.subscribe("/laboven/configA", handleMessage_configA);
  client.subscribe("/laboven/configB", handleMessage_configB);
}

void startmDNS() { // Start the mDNS responder
  debugA("Starting mDNS responder ...");

  if (!MDNS.begin(mdnsName)) {             // 
    debugE("Error setting up MDNS responder!");
  } else {
    debugA("mDNS responder started - http://%s.local", mdnsName);  
  }
}