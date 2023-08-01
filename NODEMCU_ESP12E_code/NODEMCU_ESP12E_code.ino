// Example Arduino/ESP8266 code to upload data to Google Sheets
// Follow setup instructions found here:
// https://github.com/StorageB/Google-Sheets-Logging
// 
// email: StorageUnitB@gmail.com


#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "HTTPSRedirect.h"
#include "DHTesp.h"
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "C:\Users\pbrug\Documents\01. PEB Personal\Credenciais Cloud\SecurityGit\credentials.h"

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

// parameters
float targetTempA = 22;
float targetTempB = 22;
int max_duty_cycleA = 127;  // =50% ->  [12V (power supply) - 2V (L298N voltage drop)] * max_duty_cycle = 5V
int max_duty_cycleB = 127;  // =50% ->  [12V (power supply) - 2V (L298N voltage drop)] * max_duty_cycle = 5V
int time_delay = 5000;
float Kp = 1;
float Kd = 0;
float Ki = 0;

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

int powerA = 0;
int powerB = 0;

// Enter command (insert_row or append_row) and your Google Sheets sheet name (default is Sheet1):
String payload_base =  "{\"command\": \"insert_row\", \"sheet_name\": \"DataLog\", \"values\": ";
String payload = "";

// Google Sheets setup (do not edit)
const char* host = "script.google.com";
const int httpsPort = 443;
const char* fingerprint = "";
String url = String("/macros/s/") + GScriptId + "/exec";
HTTPSRedirect* client = nullptr;

// Declare variables that will be published to Google Sheets
int value0 = 0;
float value1 = 0;
float value2 = 0;
float value3 = 0;
float value4 = 0;
float value5 = 0;
float value6 = 0;
const char* value7 = "...";

// Declare reading variables
float humidityA = 0;
float temperatureA = 0;
float humidityB = 0;
float temperatureB = 0;
int minsampleperiod = 2000;

// delay control variables
float elapsedTime, currentTime, previousTime;

// ------------------------------------------
// SETUP FUNCTION
// ------------------------------------------

void setup() {
  // Serial Communication
  Serial.begin(9600);        
  delay(10);

  debugA("\n \n Booting MCU - v1.5 in 10 seconds ...\n \n");
  delay(10000);

  // init EEPROM object 
  // to read/write wifi configuration.
  EEPROM.begin(512);
  readWifiConf();

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

  // Onboard LED
  pinMode(LED_BUILTIN, OUTPUT);  // GPIO 16

  // Set Sensor A and B
  debugV("Setting up sensors...");
  dhtA.setup(DHTpin1, DHTesp::DHT11); //for DHT11 Connect DHT sensor to GPIO 14
  dhtB.setup(DHTpin2, DHTesp::DHT11); //for DHT11 Connect DHT sensor to GPIO 12
  minsampleperiod = max(dhtA.getMinimumSamplingPeriod(), dhtB.getMinimumSamplingPeriod())/1000;

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

  // Use HTTPSRedirect class to create a new TLS connection
  debugV("Connecting to %s", host);

  client = new HTTPSRedirect(httpsPort);
  client->setInsecure();
  client->setPrintResponseBody(true);
  client->setContentTypeHeader("application/json");
  
  // Try to connect for a maximum of 10 times
  bool flag = false;
  for (int i=0; i<10; i++){ 
    int retval = client->connect(host, httpsPort);
    if (retval == 1){
       flag = true;
       debugV(" - Connected successfully");
       break;
    }
    else
      debugI("Connection failed. Retrying...");
  }
  if (!flag){
    debugE("Could not connect to %s", host);
    return;
  }
 
  // GET data from google sheet - initialize target variables
  get_data_from_sheets(client);
  previousTime = millis(); 

  // clean up
  delete client;    // delete HTTPSRedirect object
  client = nullptr; // delete HTTPSRedirect object

}

// ------------------------------------------
// MAIN LOOP
// ------------------------------------------

void loop() {

  // Give processing time for ArduinoOTA.
  ArduinoOTA.handle();

  // Give processing time for the webserver.
  server.handleClient();

  // Create HTTP client
  static bool flag = false;
  if (!flag){
    client = new HTTPSRedirect(httpsPort);
    client->setInsecure();
    flag = true;
    client->setPrintResponseBody(true);
    client->setContentTypeHeader("application/json");
  }
  if (client != nullptr){
    if (!client->connected()){
      client->connect(host, httpsPort);
      // it seems to me that a check if the connection is ok
    }
  }
  else{
    debugE("Error creating HTTPSRedirect object!");
  }

  // After each time_delay log data and retrive parameters from google sheet
  currentTime = millis();      // get current time
  elapsedTime = (currentTime - previousTime) / 1000; 

  if (elapsedTime > max(time_delay, minsampleperiod)) {
    // Turn on LED to indicate processing
    //digitalWrite(LED_BUILTIN, LOW);  

    // Get data from sensors
    get_data_from_sensors();

    // very dummy controller
    set_heater_power();
    
    // GET data from google sheet
    get_data_from_sheets(client);

    // POST data into google sheet 
    post_data_to_sheets(client);

    // Turn off LED
    //digitalWrite(LED_BUILTIN, HIGH);  

    previousTime = currentTime;      // the last "trigger time" is stored 
  }

  // RemoteDebug handle
  debugHandle();
  
  // a delay between each cycle - to be evaluated .. if it is really required  
  delay(500);
}

// -------------------------------------------------------------------
// MAIN LABORATORY OVEN FUNCTIONS (to be replace for a class+methods)
// -------------------------------------------------------------------

void get_data_from_sheets(HTTPSRedirect * client){

  debugI("Getting data from Google Sheet...");

  if(client->GET(url, host, false)){ 
    String json_response = client->getResponseBody();

 // Decode JSON/Extract values
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, json_response);  

    targetTempA = doc["targetA"];
    targetTempB = doc["targetB"];
    time_delay = doc["delay"];
    max_duty_cycleA = int(doc["max_duty_cycleA"]);
    max_duty_cycleB = int(doc["max_duty_cycleB"]);
    Kp = doc["Kp"];
    Kd = doc["Kd"];
    Ki = doc["Ki"];
    
    debugV(" - Data >> Target A:%f\tTarget B:%f\tDelay:%d\tDuty A:%d\tDuty B:%d\n", targetTempA, targetTempB, time_delay, max_duty_cycleA, max_duty_cycleB);
    debugV(" - Data >> Kp:%f\tKd:%f\tKi:%f\n", Kp, Kd, Ki);

  }
}

void post_data_to_sheets(HTTPSRedirect * client){

  debugI("Publishing data into Google Sheets...");  

  // Update counter
  value0 ++;
  value1 = temperatureA;
  value2 = temperatureB;
  value3 = powerA/max_duty_cycleA;
  value4 = powerB/max_duty_cycleB;
  value5 = targetTempA;
  value6 = targetTempB;
  // value 7

  // prepare payload
  payload = payload_base + "\"" + value0 + "," + value1+ "," + value2 + "," + value3 + "," + value4 + "," + value5 + "," + value6 + "," + value7 +"\"}";
  
  if(client->POST(url, host, payload, false)){ 
    // Published data to Google Sheets
    debugV(" - Payload >> %s\n", payload.c_str());
  }
  else{
    debugE(" - Error while connecting\n");
  }
}

void get_data_from_sensors(){
  
  debugI("Retrieve data from sensor...");
  
  // Retrieve data from sensor A
  humidityA = dhtA.getHumidity();
  temperatureA = dhtA.getTemperature() - 1;


  // Retrieve data from sensor B
  humidityB = dhtB.getHumidity();
  temperatureB = dhtB.getTemperature();

  // print status
  debugV(" - Sensor A >> Status:%s\tHumidity:%f\tTemperature:%f",  dhtA.getStatusString(), humidityA, temperatureA);
  debugV(" - Sensor B >> Status:%s\tHumidity:%f\tTemperature:%f\n",  dhtB.getStatusString(), humidityB, temperatureB);
}

void set_heater_power(){

  debugI("Setting PWM signals for heater element power...");

  if (targetTempA > temperatureA) {
    powerA = max_duty_cycleA;
  } else {
    powerA = 0;
  }
  analogWrite(ENA, powerA);

  if (targetTempB > temperatureB) {
    powerB = max_duty_cycleB;
  } else {
    powerB = 0;
  }
  analogWrite(ENB, powerB);

  // print status
  debugV(" - Power >> Heater A:%d\tHeater B:%d\n", powerA, powerB);
}


// -------------------------
// OTA CODE AND SAFEGUARDS
// -------------------------

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
  debugI("Connecting to WiFi - SSID:%s", wifiConf.wifi_ssid);

  WiFi.mode(WIFI_STA); // station mode
  WiFi.begin(wifiConf.wifi_ssid, wifiConf.wifi_password);

  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    debugV(" - Connected with IP: %s\n", WiFi.localIP());
    return true;
  } else {
    debugE(" - Connection to WiFi Failed!\n");
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
