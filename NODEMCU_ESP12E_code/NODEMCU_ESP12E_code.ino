// Laboratory Oven control system - Version 3
// - Adafruit.io service 
// - PID control

#include <Arduino.h> 
#include "C:\Users\pbrug\Documents\01. PEB Personal\Credenciais Cloud\SecurityGit\credentials.h"
#include <ESP8266WiFi.h>

#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h> 

#include <ArduinoJson.h>
#include "DHTesp.h"
#include <PID_v1.h>

// DISABLE DEBUG FOR PRODUCTION
//#define DEBUG_DISABLED true

// SELECT BETWEEN SERIAL or NETWORK DEBUG
#define REMOTE_DEBUG

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
#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, wifi_ssid, wifi_password);
AdafruitIO_Feed *feed_tempA = io.feed("laboven.tempa");
AdafruitIO_Feed *feed_tempB = io.feed("laboven.tempb");
AdafruitIO_Feed *feed_powerA = io.feed("laboven.powera");
AdafruitIO_Feed *feed_powerB = io.feed("laboven.powerb");
AdafruitIO_Feed *feed_targetA = io.feed("laboven.targeta");
AdafruitIO_Feed *feed_targetB = io.feed("laboven.targetb");

AdafruitIO_Feed *feed_delay = io.feed("laboven.delay");
AdafruitIO_Feed *feed_configA = io.feed("laboven.configa");
AdafruitIO_Feed *feed_configB = io.feed("laboven.configb");

//AdafruitIO_Feed *feed_counter = io.feed("laboven.counter");


// Configuration for fallback access point 
// if Wi-Fi connection fails.
IPAddress AP_IP = IPAddress(10,1,1,1);
IPAddress AP_subnet = IPAddress(255,255,255,0);

// Wi-Fi connection parameters.
// It will be read from the flash during setup.
struct WifiConf {
  char wifi_ssid[50];
  char wifi_password[50];
  // Make sure that there is a 0 
  // that terminatnes the c string
  // if memory is not initalized yet.
  char cstr_terminator = 0; // make sure
};
WifiConf wifiConf;

// Web server for editing configuration.
// 80 is the default http port.
ESP8266WebServer server(80);

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

// variables do retrieve and parse data from Adafruit messages
// for some unknown (yet) reason, the receive string has maximum length=52
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
   
  // init EEPROM object 
  // to read/write wifi configuration.
  EEPROM.begin(512);
  readWifiConf();

  // Connect to wifi or setup acess point in case of failure
  if (!connectToWiFi()) {
    setUpAccessPoint();
  }
  setUpWebServer();
  setUpOverTheAirProgramming();

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


  // Connect to Adafruit.io
  debugI("Connecting to Adafruit IO");

  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    debugV("Trying ...");
    delay(500);
  }

  // we are connected
  debugI("Connected to Adadruit.io: %s", io.statusText());

  // subscribe
  feed_delay->onMessage(handleMessage_delay);
  feed_configA->onMessage(handleMessage_configA);
  feed_configB->onMessage(handleMessage_configB);

  // get last published values
  feed_delay->get();
  feed_configA->get();
  feed_configB->get();

  // Set Sensor A and B
  debugV("Setting up sensors...");
  dhtA.setup(DHTpin1, DHTesp::DHT11); //for DHT11 Connect DHT sensor to GPIO 14
  dhtB.setup(DHTpin2, DHTesp::DHT11); //for DHT11 Connect DHT sensor to GPIO 12
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

  // Start mDNS
  startmDNS(); 
}

// ------------------------------------------
// MAIN LOOP
// ------------------------------------------
void loop() {

  // Debug OMM
  debugV("*** Begin Loop - Free Heap Memory:%u\t Max Block Size:%u\n", ESP.getFreeHeap(), ESP.getMaxFreeBlockSize());

  // Give processing time for ArduinoOTA.
  ArduinoOTA.handle();

  // Give processing time for the webserver.
  server.handleClient();

  // io.run(); is required for all sketches to keep the client connected to io.adafruit.com, and processes any incoming data.
  io.run();  

  // After each time_delay log retrieve parameters from google sheet and write log
  currentTime = millis();                           // get current time
  elapsedTime = (currentTime - previousTime)/1000;  // in seconds

  if (elapsedTime > time_delay) {
    debugcounter += time_delay/60.0;

    // read and write data into influxdb
    if (WiFi.status() != WL_CONNECTED){
      debugI("Wifi was disconnected... Trying to reconnect");
      connectToWiFi();
    }
    if (WiFi.status() == WL_CONNECTED){
      publish_data_to_adafruit_io();
    }

    previousTime = currentTime;      // the last "trigger time" is stored 
  }

  // Perform controlling activities - Get data from sensors and update heater power
  get_data_from_sensors();
  set_heater_power_with_PID();
  
  // RemoteDebug handle
  debugHandle();
  MDNS.update(); 
  
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

void publish_data_to_adafruit_io(){
  // Log activity
  debugI("Publishing to Adafruit.io\n");

  feed_tempA->save(temperatureA);
  feed_powerA->save(powerA*100.0);
  feed_targetA->save(targetTempA);

  feed_tempB->save(temperatureB);
  feed_powerB->save(powerB*100.0);
  feed_targetB->save(targetTempB);

//  feed_counter->save(debugcounter);
}

void handleMessage_delay(AdafruitIO_Data *data) {
  debugI("Receiving published feed... - Delay");
  time_delay = (int) data->toFloat();
  debugV(" - New delay value: %u", time_delay);
}

void handleMessage_configA(AdafruitIO_Data *data) {
  debugI("Receiving published feed... - Config A");
  json_response = data->toString();
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

void handleMessage_configB(AdafruitIO_Data *data) {
  debugI("Receiving published feed... - Config B");
  json_response = data->toString();
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

// -------------------------
// OTA CODE AND SAFEGUARDS
// -------------------------
void startmDNS() { // Start the mDNS responder
  debugA("Starting mDNS responder ...");

  if (!MDNS.begin(mdnsName)) {             // 
    debugE("Error setting up MDNS responder!");
  } else {
    debugA("mDNS responder started - http://%s.local", mdnsName);  
  }
}

void readWifiConf() {

  debugI("Retrieving data from EEPROM...");

  // Read wifi conf from flash
  for (int i=0; i<sizeof(wifiConf); i++) {
    ((char *)(&wifiConf))[i] = char(EEPROM.read(i));
  }
  // Make sure that there is a 0 
  // that terminatnes the c string
  // if memory is not initalized yet.
  wifiConf.cstr_terminator = 0;
  
  debugV(" - EEPROM >> SSID: %s \t PASS: *******\n");
}

void writeWifiConf() {

  debugI("Writing data into EEPROM...\n");

  for (int i=0; i<sizeof(wifiConf); i++) {
    EEPROM.write(i, ((char *)(&wifiConf))[i]);
  }
  EEPROM.commit();
}

bool connectToWiFi() {
  debugA("Connecting to WiFi - SSID:%s", wifiConf.wifi_ssid);

  WiFi.mode(WIFI_STA); // station mode
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(wifiConf.wifi_ssid, wifiConf.wifi_password);

  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    debugA(" - Connected with IP: %d.%d.%d.%d\n", WiFi.localIP()[0], 
    WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);    
    return true;
  } else {
    debugA(
      " - Connection to WiFi Failed!\n");
    return false;
  }
}

void setUpAccessPoint() {
    debugI("Setting up access point...");
    debugV(" - SSID: %s\t\t PASS%s", AP_ssid, AP_password);

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(AP_IP, AP_IP, AP_subnet);
    if (WiFi.softAP(AP_ssid, AP_password)) {
      debugV(" - Access point set to IP:%s", WiFi.softAPIP());
    } else {
      debugE("Setting up access point failed!");
    }
}

void setUpWebServer() {
  server.on("/", handleWebServerRequest);
  server.begin();
}

void handleWebServerRequest() {

  debugI("Handling web server request...");
  bool save = false;

  if (server.hasArg("ssid") && server.hasArg("password")) {
    server.arg("ssid").toCharArray(
      wifiConf.wifi_ssid,
      sizeof(wifiConf.wifi_ssid));
    server.arg("password").toCharArray(
      wifiConf.wifi_password,
      sizeof(wifiConf.wifi_password));

    writeWifiConf();
    save = true;

    debugV(" - SSID: %s", server.arg("ssid"));

  }

  String message = "";
  message += "<!DOCTYPE html>";
  message += "<html>";
  message += "<head>";
  message += "<title>ESP8266 conf</title>";
  message += "</head>";
  message += "<body>";
  if (save) {
    message += "<div>Saved! Rebooting...</div>";
  } else {
    message += "<h1>Wi-Fi conf</h1>";
    message += "<form action='/' method='POST'>";
    message += "<div>SSID:</div>";
    message += "<div><input type='text' name='ssid' value='" + String(wifiConf.wifi_ssid) + "'/></div>";
    message += "<div>Password:</div>";
    message += "<div><input type='password' name='password' value='" + String(wifiConf.wifi_password) + "'/></div>";
    message += "<div><input type='submit' value='Save'/></div>";
    message += "</form>";
  }
  message += "</body>";
  message += "</html>";
  server.send(200, "text/html", message);

  if (save) {
    debugV("Wi-Fi conf saved. Rebooting...");
    delay(1000);
    ESP.restart();
  }
}

void setUpOverTheAirProgramming() {

  // Change OTA port. 
  // Default: 8266
  ArduinoOTA.setPort(8266);

  // Change the name of how it is going to show up in Arduino IDE.
  ArduinoOTA.setHostname("PEB_NODEMCU_ESP12E");

  // Re-programming passowrd. 
  // ArduinoOTA.setPassword(OTA_password);
  ArduinoOTA.setPasswordHash(OTA_password_hash);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    debugI("OTA: Start updating %s", type);
  });

  ArduinoOTA.onEnd([]() {
    debugI("OTA: End update");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    debugV("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    debugE("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      debugE(" - Auth Failed\n");
    } else if (error == OTA_BEGIN_ERROR) {
      debugE(" - Begin Failed\n");
    } else if (error == OTA_CONNECT_ERROR) {
      debugE(" - Connect Failed\n");
    } else if (error == OTA_RECEIVE_ERROR) {
      debugE(" - Receive Failed\n");
    } else if (error == OTA_END_ERROR) {
      debugE(" - End Failed\n");
    }
  });

  ArduinoOTA.begin();
  debugI("OTA: Ready");
}
