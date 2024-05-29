#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <TinyGPS++.h>

//random location 27.716291, 85.356304
//q-block location 12.973696, 79.163681

//PHONE NUMBER
const String PHONE_NUMBER = "+918276952451";

//INITIALIZATION OF SIM 
#define RX_SIM_PIN 4
#define TX_SIM_PIN 3

SoftwareSerial sim800(RX_SIM_PIN,TX_SIM_PIN);

//INITIALIZATION OF GPS
#define RX_GPS_PIN 9
#define TX_GPS_PIN 8

//INITIALIZATION
AltSoftSerial neogps;
TinyGPSPlus gps;

//INITIALIZATION OF BUZZER
#define BUZZER 13

//ALARM 
int buzzer_timer = 0;
bool alarm = false;
boolean send_alert_once = true;

// PARAMETER SIZE IN METER
const float maxDistance = 5;

//INITIAL LOCATION
float initialLatitude = 27.716291;
float initialLongitude = 85.356304;

float latitude, longitude;

//FUNCTION INTIALIZE
void gpsLocationFetch(float& latitude, float& longitude);
float calculateDistance(float flat1, float flon1, float flat2, float flon2);
float calculateDistance(float flat1, float flon1, float flat2, float flon2);
void sendAlert();

//SETUP FUNCTION
void setup()
{
  Serial.println("Arduino serial initialize");
  Serial.begin(9600);

  Serial.println("SIM800L serial initialize");
  sim800.begin(9600);

  Serial.println("NEO6M serial initialize");
  neogps.begin(9600);

  pinMode(BUZZER, OUTPUT);

  sim800.println("AT"); //Check GSM Module
  delay(1000);
  sim800.println("ATE1"); //Echo ON
  delay(1000);
  sim800.println("AT+CPIN?"); //Check SIM ready
  delay(1000);
  sim800.println("AT+CMGF=1"); //SMS text mode
  delay(2000);
  sim800.println("AT+CNMI=1,1,0,0,0"); /// Decides how newly arrived SMS should be handled
  delay(1000);
  delay(10000);
  buzzer_timer = millis();
}


//LOOP FUNCTION
void loop()
{
  gpsLocationFetch(latitude, longitude);

  float distance = calculateDistance(latitude, longitude, initialLatitude, initialLongitude);

  Serial.print("Latitude= "); Serial.println(latitude, 6);
  Serial.print("Longitude= "); Serial.println(longitude, 6);
  Serial.print("initialLatitude= "); Serial.println(initialLatitude, 6);
  Serial.print("initialLngitude= "); Serial.println(initialLongitude, 6);
  Serial.print("current Distance= "); Serial.println(distance);

  // Set alarm on?
  if(distance > maxDistance) {

    if(send_alert_once == true){
      digitalWrite(BUZZER, HIGH);
      sendAlert();
      alarm = true;
      send_alert_once = false;
      buzzer_timer = millis();
    }
  }
  else{
    send_alert_once = true;
  }

  // Handle alarm
  if (alarm == true) {
    if (millis() - buzzer_timer > 5000) {
      digitalWrite(BUZZER, LOW);
      alarm = false;
      buzzer_timer = 0;
    }
  }
 
  while(sim800.available()){
    Serial.println(sim800.readString());
  }

  while(Serial.available())  {
    sim800.println(Serial.readString());
  }

}

//FUNCTION TO CALCULATE DISTANCE BETWEEN TWO COORDINATE POINTS
float calculateDistance(float flat1, float flon1, float flat2, float flon2) {

  // temporary variables initililzation
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  // Calculation
  diflat  = radians(flat2-flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2)-(flon1));

  dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
  dist_calc2 = cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0);
  dist_calc2*=sin(diflon/2.0);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
  
  dist_calc*=6371000.0; 

  return dist_calc;
}


//FETCHING GPS LOCATION
void gpsLocationFetch(float& latitude, float& longitude)
{
  // Can take up to 60 seconds
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;){
    while (neogps.available()){
      if (gps.encode(neogps.read())){
        newData = true;
        break;
      }
    }
  }
  
  if (newData)
  {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    newData = false;
  }
  else {
    Serial.println("GPS DATA IS NOT AVAILABLE");
    latitude = 0;
    longitude = 0;
  }
}


void sendAlert()
{
  String sms_data;
  sms_data = "The patient is outside the parameter.\r";
  sms_data += "http://maps.google.com/maps?q=loc:";
  sms_data += String(latitude) + "," + String(longitude);

  sim800.print("AT+CMGF=1\r");
  delay(1000);
  sim800.print("AT+CMGS=\""+PHONE_NUMBER+"\"\r");
  delay(1000);
  sim800.print(sms_data);
  delay(100);
  sim800.write(0x1A); //ascii code for ctrl-26 //sim800.println((char)26); //ascii code for ctrl-26
  delay(1000);
  Serial.println("SMS Sent Successfully.");
}

