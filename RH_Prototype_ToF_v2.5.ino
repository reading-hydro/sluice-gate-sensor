/*
 Program for ESP32
 Periodically reads from one Time-Of-Flight module wired via I2C and publishes the result over a WiFi network to an MQTT server topic
 Each reading comprises multiple read attempts, validated and then an average result is posted along with metadata to enable other analysis
 Calculates a 'gate raise' for a lifting gate that rises towards the fixed sensor mounting position above the moving parts (you can modify this for your setup)
 
 Produces either a valid response, optinally with raw data included:
{
  "sensor": "sluice_2",
  "current_timestamp": "2022-06-20T20:11:32Z",
  "status": 1,
  "gate_raised_mm": -211,
  "sample_mean": 1651,
  "sample_count": 24,
  "sample_std_dev": 14
    "raw_data": [
    1582,
    1609
  ]
}

 OR, an inadequate set of readings produces:
 
 {
  "sensor": "sluice_2",
  "current_timestamp": "2022-06-20T20:08:37Z",
  "status": 0
}

Definition of fields:
  "sensor": a fixed value
  "current_timestamp": a UTC timestamp in ISO8601 format: yyyy-mm-ddThh:mm:ddZ
  "status": boolean 1 for successful reading and 0 for failed reading 
  "gate_raised_mm": a fixed (calibration) distance subtracted from the sample mean
  "sample_mean": the mean average of the valid readings
  "sample_count": the number of valid readings used
  "sample_std_dev": one standard deviation from the mean of the valid readings
  "raw_data": [  ]  optional array of raw data values
   
*/
//#define DEBUG 1 //must be enabled to see any serial out logging. 
//#define DEBUG1 1 //WiFi and MQTT connectivity logging. 
//#define DEBUG2 1 //logging of detailed readings. 

#include "secrets.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP32Time.h>
ESP32Time rtc;
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;//To use daylight saving time, set it to 3600. Otherwise, set it to 0. We use UTC throughout this program, hence 0.

const char ssid[] = SECRET_SSID;    //  your network SSID (name)
const char password[] = SECRET_PASSWORD;   // your network password

/* this is the MDNS name of the MQTT Server */
const char* serverHostname = SERVER_HOSTNAME;
int serverPort = SERVER_PORT;

/* create an instance of WiFiClientSecure */
WiFiClientSecure espClient;
PubSubClient client(espClient);

/* Enter your MQTT PUBLISH topic here */
#define PUB_TOPIC  "hydro-test"
/* Set to 1 if you want raw data on the output */
#define RAW_DATA_POST 0
/* Set how many readings you want to perform per interval */
#define NUM_READS 30
/*Set how many seconds you want to wait between each posting */
#define WAIT_INTERVAL 450
/* Set your sensor name in output format*/
#define SENSOR_NAME "sluice_2"
/* Set the height that the sensor is mounted in mm to calibrate */
#define SENSOR_HEIGHT 1440
/* Set the I2C pin used for the SDA connection to the sensor */
#define I2C_SDA 23
/* Set the I2C pin used for the SCL connection to the sensor */
#define I2C_SCL 22

// json document code
#include <ArduinoJson.h>
// allocate the memory for the document
const size_t CAPACITY = JSON_OBJECT_SIZE(NUM_READS+50);//put number of key-vaue pairs in (n)
StaticJsonDocument<CAPACITY> doc;

// global program variables
int reading[NUM_READS];
char msg_out[CAPACITY];
int counter = 0;
int ctReads =0;
float stDev = 0;
float number = 0;
float gate_calibration = SENSOR_HEIGHT;// the default height that the sensor is mounted in mm

//
// Time of Flight Sensor settings
//
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor;
/* Uncomment this line to use long range mode. This
   increases the sensitivity of the sensor and extends its
   potential range, but increases the likelihood of getting
   an inaccurate reading because of reflections from objects
   other than the intended target. It works best in dark
   conditions.*/
#define LONG_RANGE
// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
//#define HIGH_SPEED
// - higher accuracy at the cost of lower speed
#define HIGH_ACCURACY

//
//  SETUP RUNS AT STARTUP
//
void setup() {

 #if defined DEBUG || defined DEBUG1 || defined DEBUG2
  Serial.begin(115200);
  delay(500);
 #endif

 espClient.setInsecure();
  /* configure the MQTT server with name or IPaddress and port */
 client.setServer(serverHostname, 8883);
  
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(500);
  Wire.begin(I2C_SDA, I2C_SCL);
  sensor.setTimeout(1500);

// starts the time of flight sensor
  if (!sensor.init())
  {
     #ifdef DEBUG1
       Serial.println("Failed to detect and initialize sensor!");
     #endif
    delay (10000); //general pause between readings
    ESP.restart();//Wipes memory totally and restarts ESP completely
    while (1) {}
  }else{
    #ifdef DEBUG1
      Serial.println("Initialized sensor");
    #endif
  }

// sets time of flight settings

 #if defined LONG_RANGE
   // lower the return signal rate limit (default is 0.25 MCPS)
   sensor.setSignalRateLimit(0.1);
   // increase laser pulse periods (defaults are 14 and 10 PCLKs)
   sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
   sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
 #endif

 #if defined HIGH_SPEED
    // reduce timing budget to 20 ms (default is about 33 ms)
   sensor.setMeasurementTimingBudget(20000);
 #elif defined HIGH_ACCURACY
    // increase timing budget to 200 ms
    sensor.setMeasurementTimingBudget(200000);
 #endif

// set up the onboard time clock
  if(setTime() > 0) {
    #ifdef DEBUG
      Serial.println("Clock set OK");
      #ifdef DEBUG1
        Serial.println( rtc.getTime("%A, %B %d %Y %H:%M:%S") + "    Epoch time: " + String(rtc.getEpoch()));   // (String) returns time with specified format followed by Epoch timestamp
        Serial.println("ISO Timestamp: " + rtc.getTime("%F") + "T" + rtc.getTime("%T") + "Z");   // Prints time with specified format 
      #endif
      delay(200);// Tiny delay to finish any actions
    #endif
  }else{
    #ifdef DEBUG
      Serial.println("Clock isn't set properly. Restarting ESP");
      delay(200);// Tiny delay to finish any actions
    #endif
    ESP.restart();//Wipes memory totally and restarts ESP completely 
  }

 delay(200);// Tiny delay to finish any actions 
}
//
//  END OF SETUP
//
void loop() {
/*
//  MAIN ROUTINE
*/
/* takes the readings and performs statistical analysis */
  number = take_reading(); 
  /* generates the JSON output document using the above data output and puts it into msg_out */
  formatDoc();
  /* Joins the WiFi and connects to the remote MQTT server. If not connected, will try to reconnect again */
   if (!client.connected()) {
     joinWiFi();
     mqttconnect();
  }
  /* Publishes the message onto the topic */ 
  client.publish(PUB_TOPIC, msg_out);
  /* Pause between readings, as defined at the top */
  delay (WAIT_INTERVAL * 1000); 
  /* Restarts the ESP completely, thus clearing all memory and ensuring a new NTP time is set */
   if ( millis() > (60*60*1000)){ // Calculates once an hour
     delay(200);// Tiny delay to finish any actions
     ESP.restart();
   }
}
//
// Takes the readings and performs statistical analysis
//
float take_reading() { // take average of NUM_READS readings
  float numRet = 0;
  //Take readings
  for (int r = 0; r < NUM_READS; r++) { // loop for each digit
     #ifdef DEBUG2
       Serial.print("Reading: " + String(r) + "  : ");
     #endif
       delay (100);
       reading[r] = sensor.readRangeSingleMillimeters();  // reads distance
     #ifdef DEBUG2
       if (sensor.timeoutOccurred()) {
         Serial.print(" TIMEOUT");
       } else {
         Serial.print(String(reading[r]));
       }
       Serial.println();
     #endif
       
     delay (100);
    }
  // Analyse readings
   ctReads =0;
   float readingTotal =0;
   float sqDevSum = 0.0;
    for (int r = 0; r < NUM_READS; r++) { // loop for each digit
     #ifdef DEBUG2
         Serial.print("Analyse Reading: " + String(r) + "  : ");
     #endif
     if (reading[r] > 8000 || reading[r] < 3){ // sensor max value seems to be 8190 for infinity or 65000 for timeout (duplicate this below!)
       //do nothing
        #ifdef DEBUG2
          Serial.println("Invalid");
        #endif 
       }else{
         #ifdef DEBUG2
           Serial.println("OK");
         #endif
        ctReads++ ;
        readingTotal = reading[r] + readingTotal ;
       }
    }
    if (ctReads >(NUM_READS/3)){ // ensures at least 30% of readings were valdi to returna  result
      numRet = readingTotal / float(ctReads);// this is the mean average

      // Calculating the standard devaiation. 
      // Firstly, sum the squares of the differences from the mean (only if the reading is valid - keep in line with the above - improve this by avoiding duplicating logic)
 
      for(int i = 0; i < NUM_READS; i++) {
          if (reading[i] > 8000 || reading[i] < 3){ // if invalid reading
          }else{
            sqDevSum += pow((numRet - float(reading[i])), 2); // pow(x, 2) is x squared.
          }
      }
      // FIND THE MEAN OF sqDevSum and TAKE THE SQUARE ROOT OF THAT to get  STANDARD DEVIATION. and round it too
      stDev = round(sqrt(sqDevSum/float(ctReads)));
      // this is in units of sensor readings (mm)
  
      #ifdef DEBUG
         Serial.println("Average in-range reading: " + String(numRet));
         Serial.println("Standard deviation of readings: " + String(stDev));
      #endif
    }else{
       numRet = 0;
    }
    
  return (round(numRet));// round the result for outputting etc
}
//
//
//
inline void formatDoc(){
  // reset the JSON document
    doc.clear();// clears document and releases memory. Better to destroy the document and recreate it however!
  // Add default values in the JSON document
    doc["sensor"] = SENSOR_NAME;
  // Creates a timestamp string as efficiently as possible (because we generally avoid Strings)
    String timeStamp ;
    timeStamp.reserve(21);// We reduce the number of allocations by allocating a buffer of the specified size. If we reserve enough room, the String doesn’t need to reallocate the buffer when we call +=.
    // To reduce the number of allocations, we avoid the + operator. The += operator modifies the left-side argument instead of creating a new String
    timeStamp += rtc.getTime("%F");// retrieves the date in yyyy-mm-dd format, total 10 chrs
    timeStamp += "T";
    timeStamp += rtc.getTime("%T"); //retrieves the time in hh:mm:ss format, total 8 chrs
    timeStamp += "Z";

    #ifdef DEBUG1
       Serial.println("Current timestamp = " + timeStamp);
    #endif
    doc["current_timestamp"] = timeStamp.c_str();

    // populate document with successful readings
    if (number >0 ) {
    int gate_height = round(gate_calibration - number);
     doc["status"] = 1;
     doc["gate_raised_mm"] = gate_height;
     doc["sample_mean"] = number;
     doc["sample_count"] =  ctReads;
     doc["sample_std_dev"] =  stDev;
// ISSUE: Only populated data on first loop, needs re-initialising or more space allocation or document destruction or something like that
    if (RAW_DATA_POST){
      JsonArray data = doc.createNestedArray("raw_data");//create array of actual readings
      for (int m = 0; m < NUM_READS; m++) { // loop for each digit
        data.add(reading[m]);    
      }
    }
    } else {
      doc["status"] = 0;
    }
  //output json, optionally display on Serial
   int size_out = measureJson(doc);
   serializeJson(doc, msg_out); // this is the straight output without pretty format (returns, indents etc)
   #ifdef DEBUG
     Serial.println("Publishing to: " + String (PUB_TOPIC) + "  message: ");
     serializeJsonPretty(doc, Serial);// this is the easy-to-read output
     Serial.println();
     Serial.println("Total bytes used: " + String (size_out));
     Serial.println();
   #endif 
}
//
//
//
long joinWiFi(){
 #ifdef DEBUG1
  delay(500);
  //connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to WiFi network ");
  Serial.println(ssid);
 #endif

  int waitCount = 0; // for counting each connection attempt
  if(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, password);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      #ifdef DEBUG1
         Serial.print("Connecting");
      #endif
      while(WiFi.status() != WL_CONNECTED && waitCount<100){ // waits for connection and times out at 10 seconds
        #ifdef DEBUG1
          Serial.print(".");
        #endif
        delay(200);// Repeatable unit of delay time while waiting for connection
      waitCount++; // adds 1 to the wait count
     }
  if (WiFi.status() == WL_CONNECTED){
//    WiFi is connected
//    client.setCACert(ca_cert); // for secure connections only
    delay(200);
    long rssi = WiFi.RSSI();
    #ifdef DEBUG1
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print("Netmask: ");
      Serial.println(WiFi.subnetMask());
      Serial.print("Gateway: ");
      Serial.println(WiFi.gatewayIP());
      Serial.print("RSSI:");
      Serial.println(rssi);
    #endif
    return rssi;
  } else {
      #ifdef DEBUG
          Serial.println("  WIFI CONNECTION FAIL. Return 0");
      #endif
     return 0;//sets value to zero if no connection was established
  }
 }
}
//
//
//
void mqttconnect() {
  /* Loop until reconnected */
  while (!client.connected()) {
    #ifdef DEBUG1
       Serial.print("MQTT connecting ...");
    #endif
    /* client ID */
    String clientId = "my_test_client_identifier";
    /* connect now */
    if (client.connect(clientId.c_str(),SECRET_MQTT_USER,SECRET_MQTT_PASSWORD)) {
     #ifdef DEBUG1
       Serial.println("MQTT connected");
     #endif
    }
  }
}
//
// SET TIME SECTION
//
inline unsigned long int setTime(){
 // digitalWrite(RED_LED, LOW);   // switches the current to red LED on to indicate time set
  #ifdef DEBUG
    Serial.println("   setTime():");
  #endif
//checks we're on WiFi first
  if(WiFi.status() != WL_CONNECTED){
   long w = joinWiFi();
   if (w == 0){
    // Do something when it fails? Restart?
    #ifdef DEBUG1
        Serial.println("!!!!!!!!!  NO WIFI CONNECTION  !!!!!!!");
    #endif
    return 0;
   }
  }
  // We have a network connection 
/*---------set with NTP---------------*/
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      struct tm timeinfo;
      if (getLocalTime(&timeinfo)){
        rtc.setTimeStruct(timeinfo); 
      }
     #ifdef DEBUG1
      Serial.println("Time Configured");
    #endif 
    while(!getLocalTime(&timeinfo)){ 
     // Wait until we can read the time from the ESP
     delay(100);// Time to allow time clock to be updated
    // Serial.println(rtc.getEpoch());  
     #ifdef DEBUG1
       Serial.println("Reading time ...");
     #endif 
    }
   #ifdef DEBUG1
      Serial.println("Time read OK");
   #endif 
   return 1;
}
