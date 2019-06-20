#include "Wire.h"
#include "wiring_private.h"

#define Serial SerialUSB

//Global vatiable to store start time in seconds of the mission
int startSeconds = 0;

#define Wire Wire1

TwoWire Wire1(&sercom2, 4, 3); //SDA = 4, SCL = 3


#define DS3231_I2C_ADDRESS 0x68
//======================>> HELPER FUNCTIONS for readDS3231time()==============================================
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}
//======================>> Setup and loop ==============================================
void setup()
{
  Wire.begin();
  Serial.begin(9600);
  // set the initial time here:
  // DS3231 seconds, minutes, hours, day, date, month, year
  // setDS3231time(30,42,21,4,26,11,14);

  pinPeripheral(4, PIO_SERCOM_ALT);
  pinPeripheral(3, PIO_SERCOM_ALT);

  setStartTime();
}

void loop()
{
  getMissionTime(); // display the real-time clock data on the Serial Monitor,
  delay(1000); // every second
}

//======================>> RTC related functions ==============================================
/*
 * readDS3231time()
 * setStartTime()
 * getMissionTime()
 */
//Fetches data from RTC registers and store them in parameters
void readDS3231time(byte *second, byte *minute,byte *hour){
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 3);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
}

//Called by setup function to initialize startSeconds. This startSeconds is subtracted from further RTC seconds to get mission time
void setStartTime(){
  byte second, minute, hour;
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour);
  *(&startSeconds) = (*(&hour)*3600) + (*(&minute)*60) + *(&second);

//  Serial.print(hour);
//  // convert the byte variable to a decimal number when displayed
//  Serial.print(":");
//  if (minute<10)
//  {
//    Serial.print("0");
//  }
//  Serial.print(minute, DEC);
//  Serial.print(minute);
//  Serial.print(":");
//  if (second<10)
//  {
//    Serial.print("0");
//  }
//  Serial.println(second, DEC);
}

//Called by loop function to print time in seconds 
void getMissionTime(){
  byte second, minute, hour;
  // retrieve data from DS3231
  readDS3231time(&second, &minute, &hour);

  int currSeconds = 0;
  *(&currSeconds) = (*(&hour)*3600) + (*(&minute)*60) + *(&second);

//  Serial.print(hour);
//  // convert the byte variable to a decimal number when displayed
//  Serial.print(":");
//  if (minute<10)
//  {
//    Serial.print("0");
//  }
//  Serial.print(minute);
//  Serial.print(":");
//  if (second<10)
//  {
//    Serial.print("0");
//  }
//  Serial.print(second);
//  Serial.print(": \t\t");

  int missionTime = *(&currSeconds) - *(&startSeconds);
  Serial.println(missionTime);
}

