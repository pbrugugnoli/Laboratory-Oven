// Laboratory Oven control system
// Base on several libraries
// - read/write from/to google sheets: https://github.com/StorageB/Google-Sheets-Logging
// - InfluxDB
// - PID control

#include <Arduino.h> 
#include "C:\Users\pbrug\Documents\01. PEB Personal\Credenciais Cloud\SecurityGit\credentials.h"
//#include <ESP8266WiFi.h>
#if defined(ESP32)
  #include <WiFiMulti.h>
  WiFiMulti wifiMulti;
  #define DEVICE "ESP32"
#elif defined(ESP8266)
  #include <ESP8266WiFiMulti.h>
  ESP8266WiFiMulti wifiMulti;
  #define DEVICE "NodeMCU-ESP12E"
#endif

#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>

#include "HTTPSRedirect.h"
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include "DHTesp.h"
#include <ArduinoJson.h>
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
// Time zone info
#define TZ_INFO "UTC-3"

// Influx parameters
#define INFLUXDB_URL "http://192.168.27.127:8086"
#define INFLUXDB_TOKEN "pMvYfh7lmJfRhb3rBcOWlCmXtswhLOxDSEo-Z8GBx6KBXfi0ZR935B4xAKsjfSNUwgq_iX99xT7vfOQVafkVqA=="
#define INFLUXDB_ORG "PEB"
#define INFLUXDB_BUCKET "LabOven"

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient influx_client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Declare Data points = SensorData as a table with historical IOT data, and SensorConfig as configuration table 
Point dpLabA("SensorData");
Point dpLabB("SensorData");
Point dpMCU("SensorData");
//Point datapoint("LabOven");

// Declare influx query variables
String influx_query;
String field, sensor, device;
double value;


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

// Enter command (insert_row or append_row) and your Google Sheets sheet name (default is Sheet1):
char payload_base[] = "{\"command\": \"insert_row\", \"sheet_name\": \"DataLog\", \"values\": \"%10u,%5.1f,%5.1f,%5.1f,%5.1f,%5.1f,%5.1f,%2s,%10u\"}";
char payload[124] = "";

// Google Sheets setup (do not edit)
const char* host = "script.google.com";
const int httpsPort = 443;
const char* fingerprint = "";
String url = String("/macros/s/") + GScriptId + "/exec";
HTTPSRedirect* client = nullptr;
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
  Serial.begin(9600);        
  delay(10000);
   
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

  // Get time from external service
  debugV("Synchronize time...");
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");  

  // Connect to Influxdb
  debugV("Connecting to influxDB...");
  if (influx_client.validateConnection()) {
    debugV("Connected to InfluxDB: %s\n", influx_client.getServerUrl());
  } else {
    debugE("InfluxDB connection failed: %s\n", influx_client.getLastErrorMessage() );
  }

  // Add tags to the data point
  dpLabA.addTag("sensor", "BoxA");
  dpLabB.addTag("sensor", "BoxB");
  dpMCU.addTag("device", DEVICE);

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

  // GET data from google sheet (through out influxdb) - initialize target variables
  // read and write data into influxdb
  if (WiFi.status() != WL_CONNECTED){
    debugI("Wifi was disconnected... Trying to reconnect");
    connectToWiFi();
  }
  if (WiFi.status() == WL_CONNECTED){
    get_data_from_influxdb();
  }

  previousTime = millis(); 

  // Setting up PID controllers
  myPIDA.SetMode(AUTOMATIC);
  myPIDB.SetMode(AUTOMATIC);
  myPIDA.SetSampleTime(sampleRate); // Assign the sample rate of the control
  myPIDB.SetSampleTime(sampleRate); // Assign the sample rate of the control
  myPIDA.SetOutputLimits(0.0, 1.0);
  myPIDB.SetOutputLimits(0.0, 1.0);
  update_PID_settings();

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
      post_data_into_influxdb();
    }

    previousTime = currentTime;      // the last "trigger time" is stored 
  }

  // Perform controlling activities - Get data from sensors and update heater power
  get_data_from_sensors();
  set_heater_power_with_PID();
  
  // RemoteDebug handle
  debugHandle();
  
  // a delay due to DHT sensor limitations
  delay(minsampleperiod);
}

// -------------------------------------------------------------------
// MAIN LABORATORY OVEN FUNCTIONS (to be replace for a class+methods)
// -------------------------------------------------------------------
void get_data_from_sheets(HTTPSRedirect * client){

  debugI("Getting data from Google Sheet...");

  if(client->GET(url, host, false)){ 
    json_response = client->getResponseBody(); // json_response was declared as global to avoid mem fragmentation

 // Decode JSON/Extract values
    //DynamicJsonDocument doc(1024);          // doc as a global var to avoid mem fragmentation
    debugI(" - Starting deserialization...");
    deserializeJson(doc, json_response);  

    debugI(" - Starting retrieving...");
    targetTempA = doc["targetA"];
    targetTempB = doc["targetB"];
    time_delay = doc["delay"];
    max_duty_cycleA = (int) doc["max_duty_cycleA"];
    max_duty_cycleB = (int) doc["max_duty_cycleB"];

    if ( Kp_A != doc["Kp_A"] ||  Kd_A != doc["Kd_A"] || Ki_A != doc["Ki_A"]){
      flagChangeKA = true;
      Kp_A = doc["Kp_A"];
      Kd_A = doc["Kd_A"];
      Ki_A = doc["Ki_A"];
    } else {
      flagChangeKA = false;
    }

    if ( Kp_B != doc["Kp_B"] ||  Kd_B != doc["Kd_B"] || Ki_B != doc["Ki_B"]){
      flagChangeKB = true;
      Kp_B = doc["Kp_B"];
      Kd_B = doc["Kd_B"];
      Ki_B = doc["Ki_B"];
    } else {
      flagChangeKB = false;
    }

    debugV(" - Data >> Target A:%f\tTarget B:%f\tDelay:%d\tDuty A:%d\tDuty B:%d", targetTempA, targetTempB, time_delay, max_duty_cycleA, max_duty_cycleB);
    debugV(" - Data >> A(Kp, Ki, Kd):(%f , %f, %f)\tB(Kp, Ki, Kd):(%f , %f, %f)\n", Kp_A, Ki_A, Kd_A, Kp_B, Ki_B, Kd_B);

  }
}

void post_data_to_sheets(HTTPSRedirect * client){

  debugI("Publishing data into Google Sheets...");  

  // Update counter
  value0 ++;
  value1 = temperatureA;
  value2 = temperatureB;
  value3 = powerA;
  value4 = powerB;
  value5 = targetTempA;
  value6 = targetTempB;
  // value7 = sensorError;
  value8 = ESP.getFreeHeap(); 

  // prepare payload
  // payload = payload_base + "\"" + value0 + "," + value1+ "," + value2 + "," + value3 + "," + value4 + "," + value5 + "," + value6 + ","  + value7 + ","  + value8 +"\"}";
  sprintf(payload, payload_base, value0, value1, value2, value3, value4, value5, value6, value7, value8);

  if(client->POST(url, host, payload, false)){ 
    // Published data to Google Sheets
    debugV(" - Payload >> %s\n", payload);
  }
  else{
    debugE(" - Error while connecting\n");
  }
}

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

void post_data_into_influxdb(){
  // Log activity
  debugI("Writing into Influx\n");

  // Clear fields for reusing the point. Tags will remain the same as set above.
  dpLabA.clearFields();
  dpLabA.addField("Temperature", temperatureA);
  dpLabA.addField("Power", powerA*100.0);
  dpLabA.addField("Target", targetTempA);

  dpLabB.clearFields();
  dpLabB.addField("Temperature", temperatureB);
  dpLabB.addField("Power", powerB*100.0);
  dpLabB.addField("Target", targetTempB);

  dpMCU.clearFields();
  dpMCU.addField("Error", value7);
  dpMCU.addField("FreeHeap", ESP.getFreeHeap());
  dpMCU.addField("DebugCounter", debugcounter);  

  // Check WiFi connection and reconnect if needed
  if (wifiMulti.run() != WL_CONNECTED) {
    debugE("Wifi connection lost");
  }

  // Write point
  if (!influx_client.writePoint(dpLabA)) {
    debugE("InfluxDB LabA write failed: %s\n", influx_client.getLastErrorMessage());
  }
  if (!influx_client.writePoint(dpLabB)) {
    debugE("InfluxDB LabB write failed: %s\n", influx_client.getLastErrorMessage());
  }
  if (!influx_client.writePoint(dpMCU)) {
    debugE("InfluxDB MCU write failed: %s\n", influx_client.getLastErrorMessage());
  }
}

void get_data_from_influxdb(){
  debugI("Getting data from Influxdb...");
  targetTempA = query_influx_sensor("target", "BoxA");
  targetTempB = query_influx_sensor("target", "BoxB");
  time_delay = (int) query_influx_device("delay", "NodeMCU-ESP12E");
  max_duty_cycleA = (int) query_influx_sensor("maxduty", "BoxA");
  max_duty_cycleB = (int) query_influx_sensor("maxduty", "BoxB");

  new_Kp = query_influx_sensor("Kp", "BoxA");
  new_Ki = query_influx_sensor("Ki", "BoxA");
  new_Kd = query_influx_sensor("Kd", "BoxA");
  if ( Kp_A != new_Kp ||  Kd_A != new_Kd || Ki_A != new_Ki){
    flagChangeKA = true;
    Kp_A = new_Kp;
    Kd_A = new_Kd;
    Ki_A = new_Ki;
  } else {
    flagChangeKA = false;
  }

  new_Kp = query_influx_sensor("Kp", "BoxB");
  new_Ki = query_influx_sensor("Ki", "BoxB");
  new_Kd = query_influx_sensor("Kd", "BoxB");
  if ( Kp_B != new_Kp ||  Kd_B != new_Kd || Ki_B != new_Ki){
    flagChangeKB = true;
    Kp_B = new_Kp;
    Kd_B = new_Kd;
    Ki_B = new_Ki;
  } else {
    flagChangeKB = false;
  }  

  debugV(" - Data >> Target A:%f\tTarget B:%f\tDelay:%d\tDuty A:%d\tDuty B:%d", targetTempA, targetTempB, time_delay, max_duty_cycleA, max_duty_cycleB);
  debugV(" - Data >> A(Kp, Ki, Kd):(%f , %f, %f)\tB(Kp, Ki, Kd):(%f , %f, %f)\n", Kp_A, Ki_A, Kd_A, Kp_B, Ki_B, Kd_B);  
}

double query_influx_sensor(String field, String sensor){
  // Construct a Flux query
  influx_query = "from(bucket: \"LabOven\") |> range(start: -10y) |> filter(fn: (r) => r._measurement == \"SensorConfig\" and r.sensor == \"" + sensor + "\" and r._field == \"" + field + "\") |> last() |> yield()" ;

  // Send query to the server and get result
  FluxQueryResult influx_result = influx_client.query(influx_query);

  influx_result.next();
  double result =  influx_result.getValueByName("_value").getDouble(); 

  return result;
}

double query_influx_device(String field, String device){
  // Construct a Flux query
  influx_query = "from(bucket: \"LabOven\") |> range(start: -10y) |> filter(fn: (r) => r._measurement == \"SensorConfig\" and r.device == \"" + device + "\" and r._field == \"" + field + "\") |> last() |> yield()" ;

  // Send query to the server and get result
  FluxQueryResult influx_result = influx_client.query(influx_query);

  influx_result.next();
  double result =  influx_result.getValueByName("_value").getDouble(); 

  return result;
}


void query_influx(){
  debugI("Getting data from Influxdb...");

  // Construct a Flux query
  // influx_query = "from(bucket: \"LabOven\") |> range(start: -10y) |> filter(fn: (r) => r[\"_measurement\"] == \"SensorConfig\") |> last() |> yield()";

  // // Send query to the server and get result
  // FluxQueryResult influx_result = influx_client.query(influx_query);

  // while (influx_result.next()) {
  //   field = influx_result.getValueByName("_field").getString();
  //   sensor = influx_result.getValueByName("sensor").getString();
  //   device = influx_result.getValueByName("device").getString();
  //   value = influx_result.getValueByName("_value").getDouble(); 
  //   debugI("Field: %s = %f\t Sensor: %s\t Device: %s", field, value, sensor, device);
  // }
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
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
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
