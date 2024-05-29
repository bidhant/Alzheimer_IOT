#include <SoftwareSerial.h>
#include <TinyGPS.h>

SoftwareSerial mySerial(9, 10);
TinyGPS gps;

void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  delay(1000);
}

void loop() {
  bool newdata = false;
  unsigned long start = millis();
  
  while (millis() - start < 5000) {
    if (mySerial.available()) {
      char c = mySerial.read();
      if (gps.encode(c)) {
        newdata = true;
        break;
      }
    }
  }
  
  if (newdata) {
    gpsdump(gps);
    Serial.println();
  }
}

void gpsdump(TinyGPS &gps) {
  long lat, lon;
  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  Serial.print("http://maps.google.com/maps?q=loc:");
  printFloat(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
  Serial.print(",");
  printFloat(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
}

void printFloat(double number, int digits) {
  if (number < 0.00) {
    Serial.print('-');
    number = -number;
  }
  
  double rounding = 0.50;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.00;
  
  number += rounding;
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);
  
  if (digits > 0)
    Serial.print(".");
  
  while (digits-- > 0) {
    remainder *= 10.00;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}
