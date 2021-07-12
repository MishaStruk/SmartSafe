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

// defines pins numbers
#define trigPin 27
#define echoPin 26
#define ClosingDoorLaserPin 19
#define motionSenorPin 14
/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "<WIFI_NAME>"
#define WLAN_PASS       "<WIFI_PASS>"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME   "<AIO_USERNAME>"
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
Adafruit_MQTT_Publish distance_publish = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Distance");
Adafruit_MQTT_Publish door_publish = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/DoorOpen");
Adafruit_MQTT_Publish motion_publish = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/MotionSensor");

// Setup a feed called 'security_ONOFF_Feed' for subscribing to changes.
Adafruit_MQTT_Subscribe security_ONOFF_Feed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Security_OnOff");



//Global Variables
int pirState = LOW;
int motionSensorValue = 0;


// defines variables
long duration;
int distance;



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



void setup() {
  Serial.begin(9600); // Starts the serial communication

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(ClosingDoorLaserPin, INPUT); // Sets the ClosingDoorLaserPin as an Input
  pinMode(motionSenorPin, INPUT);  // motion sensor pin


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

}



void UpdateMotionSensor()
{
  //MOTION
  motionSensorValue = digitalRead(motionSenorPin);
  char* motionValue = "NOTDETECTED";
  if (motionSensorValue == HIGH) {

    Serial.println("Motion Detected!");
    motionValue = "DETECTED";
  }
  else {
    Serial.println("Motion Ended!");
    motionValue = "NOTDETECTED";
  }
  if (! motion_publish.publish(motionValue)) {
    Serial.println(F("Failed to Update Motion"));
  } else {
    Serial.println(F("Motion Updated"));
  }
}

void UpdateDoorSensor()
{
  // Door LASER
  int DoorClosed = digitalRead(ClosingDoorLaserPin);// read Laser sensor
  char* doorValue = "CLOSED";
  if ( DoorClosed == HIGH)
  {
    Serial.println("Door Open");
    doorValue = "OPEN";
  } else {

    Serial.println("Door Closed");
    doorValue = "CLOSED";
  }
  if (! door_publish.publish(doorValue)) {
    Serial.println(F("Failed to Update door status"));
  } else {
    Serial.println(F("Door status Updated"));
  }
}

void UpdateDistanceSensor()
{

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(10);
  if (! distance_publish.publish(distance)) {
    Serial.println(F("Failed to Update distance"));
  } else {
    Serial.println(F("Distance Updated"));
  }
}

void loop() {
  // Connection to the Cloud
  MQTT_connect();
  UpdateMotionSensor();
  delay(100);
  UpdateDoorSensor();
  delay(100);
  UpdateDistanceSensor();
  delay(5000);
}
