//message will be sent as soon as the lidar is outside of the given distance. 


#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <TinyGPS++.h>

//PHONE NUMBER
const String PHONE_NUMBER = "+919043075951";

//INITIALIZATION OF SIM 
#define RX_SIM_PIN 4
#define TX_SIM_PIN 3

SoftwareSerial sim800(RX_SIM_PIN, TX_SIM_PIN);

//INITIALIZATION OF GPS
#define RX_GPS_PIN 9
#define TX_GPS_PIN 8

//INITIALIZATION
AltSoftSerial neogps;
TinyGPSPlus gps;

//INITIALIZATION OF BUZZER
#define BUZZER 13

//ALARM 
bool alarm = false;
boolean send_alert_once = true;

// PARAMETER SIZE IN METERS
const float maxDistance = 5;

//INITIAL LOCATION
float initialLatitude = 27.716291;
float initialLongitude = 85.356304;

float latitude, longitude;

//SETUP FUNCTION
void setup() {
  Serial.begin(9600);
  sim800.begin(9600);
  neogps.begin(9600);
  pinMode(BUZZER, OUTPUT);

  // Check GSM Module
  sim800.println("AT");
  delay(1000);

  // Echo ON
  sim800.println("ATE1");
  delay(1000);

  // Check SIM ready
  sim800.println("AT+CPIN?");
  delay(1000);

  // SMS text mode
  sim800.println("AT+CMGF=1");
  delay(1000);

  // Decides how newly arrived SMS should be handled
  sim800.println("AT+CNMI=1,1,0,0,0");
  delay(1000);
}

//LOOP FUNCTION
void loop() {
  gpsLocationFetch(latitude, longitude);
  float distance = calculateDistance(latitude, longitude, initialLatitude, initialLongitude);

  Serial.print("Latitude= "); Serial.println(latitude, 6);
  Serial.print("Longitude= "); Serial.println(longitude, 6);
  Serial.print("initialLatitude= "); Serial.println(initialLatitude, 6);
  Serial.print("initialLngitude= "); Serial.println(initialLongitude, 6);
  Serial.print("current Distance= "); Serial.println(distance);

  // Set alarm on
  if (distance > maxDistance) {
    if (send_alert_once) {
      digitalWrite(BUZZER, HIGH);
      send_alert_once = false;
      alarm = true;
    }
  } else {
    send_alert_once = true;
  }

  // Handle alarm
  if (alarm && !send_alert_once) {
    // Buzzer remains activated until reset button is pressed
    // No need for time-based buzzer control
  } else {
    digitalWrite(BUZZER, LOW);
    alarm = false;
  }

  // Process incoming messages from SIM800
  while (sim800.available()) {
    Serial.println(sim800.readString());
  }

  // Pass incoming messages from Serial to SIM800
  while (Serial.available()) {
    sim800.println(Serial.readString());
  }
}

//FUNCTION TO CALCULATE DISTANCE BETWEEN TWO COORDINATE POINTS
float calculateDistance(float flat1, float flon1, float flat2, float flon2) {
  float diflat = radians(flat2 - flat1);
  float diflon = radians(flon2 - flon1);
  float dist_calc = sin(diflat / 2.0) * sin(diflat / 2.0);
  dist_calc += cos(radians(flat1)) * cos(radians(flat2)) * sin(diflon / 2.0) * sin(diflon / 2.0);
  dist_calc = 2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc));
  dist_calc *= 6371000.0;  // Radius of Earth in meters
  return dist_calc;
}

//FETCHING GPS LOCATION
void gpsLocationFetch(float& latitude, float& longitude) {
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (neogps.available()) {
      if (gps.encode(neogps.read())) {
        newData = true;
        break;
      }
    }
  }
  
  if (newData) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  } else {
    Serial.println("GPS DATA IS NOT AVAILABLE");
    latitude = 0;
    longitude = 0;
  }
}
