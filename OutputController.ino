/***************************************************
  SHENKAR - SMART SYSTEMS
  By: Michael Struk and Noam Roytman
  DATE: JUL-2021
 ****************************************************/
/************************* Settings *********************************/
//Libraries
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
//Pins Setting
#define REDLEDPIN 25
#define BLUELEDPIN 18
#define YELLOWLEDPIN 26
#define BUZZERPIN 10

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "<WIFI NAME>"
#define WLAN_PASS       "<WIFI PASS>"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME   "<AI_NAME>"
#define AIO_KEY        "<AIO_KEY>"

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

//Variables uses to update and read temp data
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>

// Setup a feed called 'security_ONOFF_Feed' for subscribing to changes.
Adafruit_MQTT_Subscribe security_ONOFF_Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Security_OnOff");
// Setup a feed called 'DoorOpen_Feed' for subscribing to changes.
Adafruit_MQTT_Subscribe DoorOpen_Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/DoorOpen");
// Setup a feed called 'MotionSensor_Feed' for subscribing to changes.
Adafruit_MQTT_Subscribe MotionSensor_Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/MotionSensor");
// Setup a feed called 'Distance_Feed' for subscribing to changes.
Adafruit_MQTT_Subscribe Distance_Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Distance");





/*************************** Sketch Code ************************************/
//Global Variables

int distance = 0;
String door_status = "CLOSED";
String motion_status = "NOTDETECTED";
bool system_status = false;
int DISTANE_IN_SAFE = g;

const char *ONOFFARRAY[2] = { "OFF", "ON" };

//Functions

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

void CheckStatusAndSetPins()
{
  if(!system_status)
  {
    digitalWrite(BLUELEDPIN, LOW);
    digitalWrite(BUZZERPIN, LOW);
    digitalWrite(REDLEDPIN, LOW);
    digitalWrite(YELLOWLEDPIN, LOW);
  }
  else
  {
    digitalWrite(YELLOWLEDPIN, HIGH);
  }
  
  if(distance >= DISTANE_IN_SAFE && system_status)
  {
      digitalWrite(BLUELEDPIN, HIGH);
  }
  else
  {
    digitalWrite(BLUELEDPIN, LOW);
  }
  
  if(door_status =="OPEN" && system_status)
  {
      digitalWrite(BUZZERPIN, HIGH);
  }
  else
  {
    digitalWrite(BUZZERPIN, LOW);
  }
  if(motion_status =="DETECTED" && system_status)
  {
      digitalWrite(REDLEDPIN, HIGH);
  }
  else
  {
    digitalWrite(REDLEDPIN, LOW);
  }
  
}

void setup() {
  Serial.begin(9600);
  pinMode(REDLEDPIN, OUTPUT); // Sets the Red led as an Output
  pinMode(BLUELEDPIN, OUTPUT); // Sets the Blue led as an Output
  pinMode(YELLOWLEDPIN, OUTPUT); // Sets the Yellow led as an Output
  pinMode(BUZZERPIN, OUTPUT); // Sets the Buzzer pin as an Output

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  Serial.println();


  // SETUP ALL THE SUBSRCIPTIONS HERE
  mqtt.subscribe(&security_ONOFF_Feed);     // Setup MQTT subscription for security feed.
  mqtt.subscribe(&DoorOpen_Feed);           // Setup MQTT subscription for Door open feed.
  mqtt.subscribe(&MotionSensor_Feed);       // Setup MQTT subscription for Motion Sensor feed.
  mqtt.subscribe(&Distance_Feed);           // Setup MQTT subscription for Distance feed.
}

void loop() {
  // Connection to the Cloud
  MQTT_connect();

  //Getting The data from the cloud
  //Getting Data
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &security_ONOFF_Feed) {
      // Getting Turn On status changed
      String val = String((char *)security_ONOFF_Feed.lastread);
      Serial.println((char *)security_ONOFF_Feed.lastread);
      if(val == "ON")
      {
        Serial.println("System turned On!");
        system_status = true;
        digitalWrite(YELLOWLEDPIN, HIGH); // Turn Yellow Pin On
      }
      else
      {
        Serial.println("System turned Off!");
        system_status = false;
        digitalWrite(YELLOWLEDPIN, LOW); // Turn Yellow Pin off
      }
    }
    if(system_status)
      {
        if (subscription == &Distance_Feed) {
          Serial.println("*****DISTANCE CHANGE**************");
          String val = String((char *)Distance_Feed.lastread);
          int value_from_cloud = val.toInt();  //Converts string to integer
          Serial.println(value_from_cloud);
          distance = value_from_cloud;
        }
        if (subscription == &DoorOpen_Feed) {
          String val = String((char *)DoorOpen_Feed.lastread);
          Serial.println((char *)DoorOpen_Feed.lastread);
          door_status = val;
        }
        if (subscription == &MotionSensor_Feed) {
          String val = String((char *)MotionSensor_Feed.lastread);
          Serial.println((char *)MotionSensor_Feed.lastread);
          motion_status = val;
        }
        
      }
  }
  CheckStatusAndSetPins();
  Serial.println("** Current Status **");
  Serial.print("System Status: ");
  Serial.println(ONOFFARRAY[system_status]);
  Serial.print("Door Status: ");
  Serial.println(door_status);
  Serial.print("Motion Sensor Status: ");
  Serial.println(motion_status);
  Serial.print("Distance from item: ");
  Serial.println(distance);
  Serial.println("********************");
  
  delay(5000); //Waiting 5 seconds for next reading and testing
}
