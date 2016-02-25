#include <TinyGPS.h>
#include<SoftwareSerial.h>

//TinyGPS gps;
//SoftwareSerial Serialg(10,11);
void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits );

void setupgps()  
{
  Serial.begin(57600);
  Serialg.begin(9600);
  //Serial1.begin(57600);  
  delay(1000);
}

void gpsdata()
{
  //Serial.println("just got inside gpsdata");
  bool newdata = false;
  if (Serialg.available()) {
    char c = Serialg.read();
   // Serial.print(c);  // uncomment to see raw GPS dat
    if (gps.encode(c)) {
      Serial.println("daaDasddsvsdafsafasdf");
      newdata = true;
    }
  }
  
  if (newdata) {
    gpsdump(gps);
    Serial.println();
  }
}

void gpsdump(TinyGPS &gps)
{
  float flat, flon; // float lat and float lon
  gps.f_get_position(&flat, &flon);
  Serial.print("$"); printFloat(flat, 7); Serial.print("#"); printFloat(flon, 7);
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}
