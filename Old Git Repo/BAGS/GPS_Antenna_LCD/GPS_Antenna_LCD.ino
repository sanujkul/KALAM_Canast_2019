#include <TinyGPS++.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 11, TXPin = 10;
static const uint32_t GPSBaud = 9600;
const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  lcd.begin(16, 2);
  lcd.print("GPS Tester");
  delay(3000);
  lcd.clear();
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read())){
      displayInfo();
      delay(1000);
      digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
    }
  if (millis() > 5000 && gps.charsProcessed() < 10){
    lcd.setCursor(0, 1);
    lcd.print("No GPS detected");
    while(true);
  }
}

void displayInfo()
{
//  if (gps.location.isValid()) {
    lcd.setCursor(0, 0);
    lcd.print("Time: " + String(millis()/1000)+"s");
    lcd.setCursor(0, 1);
    lcd.print("Number of sat: ");
    unsigned long int sat_no = gps.satellites.value();
    lcd.print(sat_no); 
//  } else {
//    lcd.setCursor(0, 1);
//    lcd.println("Invalid Data Rec.");
//  }
}
