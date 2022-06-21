# sluice-gate-sensor
Custom code for an ESP32 with time-of-flight sensor VL53L0X that supports the sluice gate measurement for readinghydro.org

The circuit diagram shows how an I2C time of flight sensor is wired to an ESP32.
It is designed for continuous 5v power, however it can be easily adapted to battery powered operation by increasing the time interval between samples and changing the sleep() statement for deep sleep. For battery operation, try the ultra low power standby boards at WWW.EZSBC.COM 

Code is written in c++ for Arduino IDE
If using Arduino IDE, ensure the sketch is uploaded and select board type of "ESP32 Dev Module"
Libraries you will need include
WiFi by Arduino version 1.2.7 or later
ArduinoJSON 6.9.14 or later
ESP32Time by fbiego version 1.0.4 or later
EspMQTT Client by Patrick Lapointe version 1.13.3 or later
PubSubClient by Nick O'Leary version 2.8.0 or later
VL53L0X by Pololu version 1.3.0 or later
WifiESP by Bruno Portaluri version 2.2.2 or later (might not be needed here)

Program summary:

Periodically reads from one Time-Of-Flight module wired via I2C and publishes the result over a WiFi network to an MQTT server topic
Each reading comprises multiple read attempts, validated and then an average result is posted along with metadata to enable other analysis
Calculates a 'gate raise' for a lifting gate that rises towards the fixed sensor mounting position above the moving parts (you can modify this for your setup)

You will also need a separate secrets.h file for your secrets, structured as:

#define SECRET_SSID "mySSID"		// replace MySSID with your WiFi network name

#define SECRET_PASSWORD "myPassword"	// replace MyPassword with your WiFi password

#define SERVER_HOSTNAME "myserver" // replace e.g. with example.org

#define SERVER_PORT nnnn // nnnn must be an integer e.g. 8883 WITHOUT quotes

#define SECRET_MQTT_USER "MyMQTTusername"      // MQTT username

#define SECRET_MQTT_PASSWORD  "MyMQTTpassword"  // MQTT password
