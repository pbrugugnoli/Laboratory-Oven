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
#include "C:\Users\pbrug\Documents\01. PEB Personal\Credenciais Cloud\SecurityGit\credentials.h"
#include <ArduinoOTA.h>


#define HOST_NAME "remotedebug"
#define USE_ARDUINO_OTA true
#include "RemoteDebug.h"        //https://github.com/JoaoLopesF/RemoteDebug
RemoteDebug Debug;



// Wi fi crendentials
// const char* ssid     = "xxx"  from crendentials file
// const char* password = "xxx"  from crendentials file

// Enter Google Script Deployment ID:
// const char *GScriptId = "xxx"  from crendentials file

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

// Declare reading variables
float humidityA = 0;
float temperatureA = 0;
float humidityB = 0;
float temperatureB = 0;

// delay control variables
float elapsedTime, currentTime, previousTime;

// ------------------------------------------
// SETUP FUNCTION
// ------------------------------------------

void setup() {
  // Serial Communication
  Serial.begin(9600);        
  delay(10);
  Serial.println('\n');

  Serial.println();
  Serial.println("Booting MCU - v1.3 in 10 seconds ...");
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
	Debug.begin(HOST_NAME); // Initialize the WiFi server
  Debug.setResetCmdEnabled(true); // Enable the reset command
	Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
	Debug.showColors(true); // Colors

  // Onboard LED
  pinMode(LED_BUILTIN, OUTPUT);  // GPIO 16

  // Set Sensor A
  Serial.println();
  Serial.println("Setting up sensor A");
  dhtA.setup(DHTpin1, DHTesp::DHT11); //for DHT11 Connect DHT sensor to GPIO 14
  
  // Set Sensor B
  Serial.println();
  Serial.println("Setting up sensor B");
  dhtB.setup(DHTpin2, DHTesp::DHT11); //for DHT11 Connect DHT sensor to GPIO 12

  // Setup L298N driver pins
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
  client = new HTTPSRedirect(httpsPort);
  client->setInsecure();
  client->setPrintResponseBody(true);
  client->setContentTypeHeader("application/json");
  
  Serial.print("Connecting to ");
  Serial.println(host);

  // Try to connect for a maximum of 5 times
  bool flag = false;
  for (int i=0; i<10; i++){ 
    int retval = client->connect(host, httpsPort);
    if (retval == 1){
       flag = true;
       Serial.println("Connected");
       break;
    }
    else
      Serial.println("Connection failed. Retrying...");
  }
  if (!flag){
    Serial.print("Could not connect to server: ");
    Serial.println(host);
    return;
  }
  else{
    // do stuff here if publish was not successful
    Serial.println("Error while connecting");
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
    }
  }
  else{
    Serial.println("Error creating client object!");
  }

  // After each time_delay log data and retrive parameters from google sheet
  currentTime = millis();      // get current time
  elapsedTime = (currentTime - previousTime) / 1000; 

  if (elapsedTime > time_delay) {
    // Turn on LED to indicate processing
    digitalWrite(LED_BUILTIN, LOW);  

    // Get data from sensors
    get_data_from_sensors();

    // very dummy controller
    set_heater_power();
    
    // GET data from google sheet
    get_data_from_sheets(client);

    // POST data into google sheet 
    post_data_to_sheets(client);

    // Turn off LED
    digitalWrite(LED_BUILTIN, HIGH);  

    previousTime = currentTime;      // the last "trigger time" is stored 
  }

  // RemoteDebug handle
  Debug.handle();
  
  // a delay between each cycle - to be evaluated .. if it is really required  
  delay(500);
}

// -------------------------------------------------------------------
// MAIN LABORATORY OVEN FUNCTIONS (to be replace for a class+methods)
// -------------------------------------------------------------------

void get_data_from_sheets(HTTPSRedirect * client){
  Serial.println("Getting data...");
  debugV("Getting data... \n");

  if(client->GET(url, host)){ 
    //String json_response = client->getResponseBody();
    String json_response = client->getResponseBody();

    DynamicJsonDocument doc(1024);
    deserializeJson(doc, json_response);  

    targetTempA = doc["targetA"];
    targetTempB = doc["targetB"];
    time_delay = doc["delay"];
    max_duty_cycleA = int(doc["max_duty_cycleA"]);
    max_duty_cycleB = int(doc["max_duty_cycleB"]);
      
    // Decode JSON/Extract values
    Serial.print("Target A: \t");
    Serial.println(targetTempA);
    Serial.print("Target b: \t");
    Serial.println(targetTempB);
    Serial.print("Delay: \t");
    Serial.println(time_delay);
    Serial.print("Max Duty Cycle A: \t");
    Serial.println(max_duty_cycleA);
    Serial.print("Max Duty Cycle B: \t");
    Serial.println(max_duty_cycleB);
    debugV("Target A: \t %f \n Target B: \t %f \n Delay: \t %d \n Duty A: \t %d \n Duty B: \t %d \n", targetTempA, targetTempB, time_delay, max_duty_cycleA, max_duty_cycleB);

  }
}

void post_data_to_sheets(HTTPSRedirect * client){
  // Update counter
  value0 ++;
  value1 = temperatureA;
  value2 = temperatureB;
  value3 = powerA/max_duty_cycleA;
  value4 = powerB/max_duty_cycleB;
  value5 = targetTempA;
  value6 = targetTempB;

  // prepare payload
  payload = payload_base + "\"" + value0 + "," + value1+ "," + value2 + "," + value3 + "," + value4 + "," + value5 + "," + value6 + "\"}";
  
  // Publish data to Google Sheets
  Serial.println("Publishing data ...");  
  Serial.println(payload);

  debugV("Publishing data ... \n %s", payload.c_str());

  if(client->POST(url, host, payload)){ 
    // do stuff here if publish was successful
  }
  else{
    // do stuff here if publish was not successful
    Serial.println("Error while connecting");
  }
}

void get_data_from_sensors(){
  // Retrieve data from sensor A
  delay(dhtA.getMinimumSamplingPeriod());
  humidityA = dhtA.getHumidity();
  temperatureA = dhtA.getTemperature();

  // Retrieve data from sensor B
  delay(dhtB.getMinimumSamplingPeriod());
  humidityB = dhtB.getHumidity();
  temperatureB = dhtB.getTemperature();

  // print status
  Serial.println("Sensor A");
  Serial.print(dhtA.getStatusString());
  Serial.print("\t");
  Serial.print(humidityA, 1);
  Serial.print("\t\t");
  Serial.println(temperatureA, 1);

  Serial.println("Sensor B");
  Serial.print(dhtB.getStatusString());
  Serial.print("\t");
  Serial.print(humidityB, 1);
  Serial.print("\t\t");
  Serial.println(temperatureB, 1);
}

void set_heater_power(){
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
  Serial.print("New PowerA: ");
  Serial.println(powerA, 0);
  Serial.print("New PowerB: ");
  Serial.println(powerB, 0);
}


// -------------------------
// OTA CODE AND SAFEGUARDS
// -------------------------

void readWifiConf() {
  // Read wifi conf from flash
  for (int i=0; i<sizeof(wifiConf); i++) {
    ((char *)(&wifiConf))[i] = char(EEPROM.read(i));
  }
  // Make sure that there is a 0 
  // that terminatnes the c string
  // if memory is not initalized yet.
  wifiConf.cstr_terminator = 0;
  
  Serial.println("EEPROM read");
  Serial.println(wifiConf.wifi_ssid);
  Serial.println("***PASS***");
}

void writeWifiConf() {
  for (int i=0; i<sizeof(wifiConf); i++) {
    EEPROM.write(i, ((char *)(&wifiConf))[i]);
  }
  EEPROM.commit();
}

bool connectToWiFi() {
  Serial.printf("Connecting to '%s'\n", wifiConf.wifi_ssid);

  WiFi.mode(WIFI_STA); // station mode
  WiFi.begin(wifiConf.wifi_ssid, wifiConf.wifi_password);

  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("Connection Failed!");
    return false;
  }
}

void setUpAccessPoint() {
    Serial.println("Setting up access point.");
    Serial.printf("SSID: %s\n", AP_ssid);
    Serial.printf("Password: %s\n", AP_password);

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(AP_IP, AP_IP, AP_subnet);
    if (WiFi.softAP(AP_ssid, AP_password)) {
      Serial.print("Ready. Access point IP: ");
      Serial.println(WiFi.softAPIP());
    } else {
      Serial.println("Setting up access point failed!");
    }
}

void setUpWebServer() {
  server.on("/", handleWebServerRequest);
  server.begin();
}

void handleWebServerRequest() {
  bool save = false;

  if (server.hasArg("ssid") && server.hasArg("password")) {
    server.arg("ssid").toCharArray(
      wifiConf.wifi_ssid,
      sizeof(wifiConf.wifi_ssid));
    server.arg("password").toCharArray(
      wifiConf.wifi_password,
      sizeof(wifiConf.wifi_password));

    Serial.println(server.arg("ssid"));
    Serial.println(wifiConf.wifi_ssid);

    writeWifiConf();
    save = true;
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
    Serial.println("Wi-Fi conf saved. Rebooting...");
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
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready");

}
