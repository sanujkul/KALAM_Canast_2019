#include <TinyGPS++.h>
//#include <SoftwareSerial.h>
/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
//SoftwareSerial ss(RXPin, TXPin);
#define ss Serial3

void setup()
{
  Serial.begin(9600);
  ss.begin(GPSBaud);
  Serial1.begin(9600);
}

void loop()
{
  
  setGPSValues();
  smartDelay(1000);                                                                           //SmartDelay Function defined seperatly

////  if (millis() > 5000 && gps.charsProcessed() < 10)
////    Serial.println(F("No GPS data received: check wiring"));
//
//  Serial.println();  
}

void setGPSValues(){
  String gpsTime = getGPSTime();
  String numSat = getGPSNumSat();
  String latitude = getGPSLat();
  String longitude = getGPSLong();
  String alt = getGPSAlt();

  Serial.print(gpsTime);
  Serial.print(",");
  Serial.print(latitude);
  Serial.print(",");
  Serial.print(longitude);
  Serial.print(",");
  Serial.print(alt);
  Serial.print(",");
  Serial.println(numSat);
}

String getGPSTime(){
  return getDateTime2(gps.time);
}

String getGPSLat(){
  return getDataInThisFormat(gps.location.lat(), gps.location.isValid(), 4);
}

String getGPSLong(){
  return getDataInThisFormat(gps.location.lng(), gps.location.isValid(), 4);
}

String getGPSAlt(){
  return getDataInThisFormat(gps.altitude.meters(), gps.altitude.isValid(), 1);
}

String getGPSNumSat(){
  return getInt2(gps.satellites.value(), gps.satellites.isValid());
}

// This custom version of delay() ensures that the gps object
// is being "fed".
void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

String getDataInThisFormat(float val, bool valid, int prec)                              //Function to print float values
{
  if (!valid)
  {
    return "-1";
  }
  else
  {
//    val = floor(10000*val)/10000;
    String toReturn = String(val);
//    Serial.println("VAL = "+String(val));
    int decimalPoint = toReturn.indexOf(".");
    toReturn = toReturn.substring(0,decimalPoint+prec+1);
    return toReturn;
  }
  smartDelay(0);
}

String getInt2(unsigned long val, bool valid)                                    //Function to print int values
{ 
//  Serial.println("VAL = "+String(val));
//  char sz[32] = "*****************";
  if (valid)
    return String(val);
  else
    return "-1";
  
  smartDelay(0);
}

String getDateTime2(TinyGPSTime &t)
{  
  if (!t.isValid())
  {
    return "-1";
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
//    return (sz);
    String s(sz); 
    return s;
  }

//  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}


