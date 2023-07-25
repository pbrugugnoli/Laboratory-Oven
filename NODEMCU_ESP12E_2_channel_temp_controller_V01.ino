// Example Arduino/ESP8266 code to upload data to Google Sheets
// Follow setup instructions found here:
// https://github.com/StorageB/Google-Sheets-Logging
// 
// email: StorageUnitB@gmail.com


#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "HTTPSRedirect.h"
#include "DHTesp.h"
#include <ArduinoJson.h>
#include "C:\Users\pbrug\Documents\01. PEB Personal\Credenciais Cloud\SecurityGit\credentials.h"

// Wi fi crendentials
// const char* ssid     = "xxx"  from crendentials file
// const char* password = "xxx"  from crendentials file

// Enter Google Script Deployment ID:
// const char *GScriptId = "xxx"  from crendentials file


// parameters
float targetTempA = 22;
float targetTempB = 22;
int time_delay = 5000;
int max_duty_cycleA = 127;  // =50% ->  [12V (power supply) - 2V (L298N voltage drop)] * max_duty_cycle = 5V
int max_duty_cycleB = 127;  // =50% ->  [12V (power supply) - 2V (L298N voltage drop)] * max_duty_cycle = 5V

// Temperature Sensors
#define DHTpin1 14   //D5 of NodeMCU is GPIO14
#define DHTpin2 12   //D6 of NodeMCU is GPIO12
DHTesp dht1;
DHTesp dht2;

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

void setup() {

  // Onboard LED
  pinMode(LED_BUILTIN, OUTPUT);  // GPIO 16

  // Serial Communication
  Serial.begin(9600);        
  delay(10);
  Serial.println('\n');

  // Set Sensor 01
  Serial.println();
  Serial.println("Setting up sensor 1");
  dht1.setup(DHTpin1, DHTesp::DHT11); //for DHT11 Connect DHT sensor to GPIO 14
  
  // Set Sensor 02
  Serial.println();
  Serial.println("Setting up sensor 2");
  dht2.setup(DHTpin2, DHTesp::DHT11); //for DHT11 Connect DHT sensor to GPIO 12

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

  // Connect to WiFi
  WiFi.begin(ssid, password);             
  Serial.print("Connecting to ");
  Serial.print(ssid); Serial.println(" ...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println('\n');
  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());

  // Use HTTPSRedirect class to create a new TLS connection
  client = new HTTPSRedirect(httpsPort);
  client->setInsecure();
  client->setPrintResponseBody(true);
  client->setContentTypeHeader("application/json");
  
  Serial.print("Connecting to ");
  Serial.println(host);

  // Try to connect for a maximum of 5 times
  bool flag = false;
  for (int i=0; i<5; i++){ 
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
 
  // clean up
  delete client;    // delete HTTPSRedirect object
  client = nullptr; // delete HTTPSRedirect object

}


void loop() {

  // Turn on LED
  digitalWrite(LED_BUILTIN, LOW);  

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

  // GET data from google sheet
  Serial.println("Getting data...");
  if(client->GET(url, host)){ 
    //String json_response = client->getResponseBody();
    String json_response = client->getResponseBody();

    DynamicJsonDocument doc(1024);
    deserializeJson(doc, json_response);  

    targetTempA = doc["target1"];
    targetTempB = doc["target2"];
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
  }

  // Get data from sensor 1
  delay(dht1.getMinimumSamplingPeriod());
  float humidity1 = dht1.getHumidity();
  float temperature1 = dht1.getTemperature();
  
  // very dummy controller
  if (targetTempA > temperature1) {
    powerA = max_duty_cycleA;
  } else {
    powerA = 0;
  }
  analogWrite(ENA, powerA);

  // Get data from sensor 2
  delay(dht2.getMinimumSamplingPeriod());
  float humidity2 = dht2.getHumidity();
  float temperature2 = dht2.getTemperature();

  if (targetTempB > temperature2) {
    powerB = max_duty_cycleB;
  } else {
    powerB = 0;
  }
  analogWrite(ENB, powerB);

  // print status
  Serial.println("Sensor 1");
  Serial.print(dht1.getStatusString());
  Serial.print("\t");
  Serial.print(humidity1, 1);
  Serial.print("\t\t");
  Serial.print(temperature1, 1);
  Serial.print("\t\t");
  Serial.println(powerA, 0);

  Serial.println("Sensor 2");
  Serial.print(dht2.getStatusString());
  Serial.print("\t");
  Serial.print(humidity2, 1);
  Serial.print("\t\t");
  Serial.print(temperature2, 1);
  Serial.print("\t\t");
  Serial.println(powerB, 0);

  // POST data into google sheet 
  // Create json object string to send to Google Sheets

  // Update counter
  value0 ++;
  value1 = temperature1;
  value2 = temperature2;
  value3 = powerA/max_duty_cycleA;
  value4 = powerB/max_duty_cycleB;
  value5 = targetTempA;
  value6 = targetTempB;

  // prepare payload
  payload = payload_base + "\"" + value0 + "," + value1+ "," + value2 + "," + value3 + "," + value4 + "," + value5 + "," + value6 + "\"}";
  
  // Publish data to Google Sheets
  Serial.println("Publishing data...");
  Serial.println(payload);
  if(client->POST(url, host, payload)){ 
    // do stuff here if publish was successful
  }
  else{
    // do stuff here if publish was not successful
    Serial.println("Error while connecting");
  }

  // Turn off LED
  digitalWrite(LED_BUILTIN, HIGH);  

  // a delay of several seconds is required before publishing again    
  delay(time_delay);
}