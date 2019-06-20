#include "Adafruit_BMP280.h"

#include <Wire.h>
//#include <SFE_BMP180.h>
#include <Servo.h>
#include <EEPROM.h>
#define BOOT 1
#define ACCENT 2
#define DEPLOYMENT 3
#define DESCENT 4
#define END 5

#define eeprom 0x50

#define test

# define DEBUG

Adafruit_BMP280 bmp; // I2C

 int oldSoftState = BOOT;
const int ldr_pin = A0;
const int buzzer_pin = A1;
//SFE_BMP180 bmp180;
int prev_Alt = 0;
int software_state = 1;
double a = 0;
long addr = 0;
Servo myservo;
int angle = 0;
byte value;

#define servo_delay 800


//http://www.hobbytronics.co.uk/eeprom-page-write
//http://www.hobbytronics.co.uk/arduino-external-eeprom

//http://www.jucetechnology.com/wordpress/simple-burglar-save-sensor-code-eeprom/

double baseline; // baseline pressure

volatile float ground_altitude = 0;
float relAltitude = 0;

void setup()
{
  initBmp();
  setGroundAltitude();
  
  int oldSoftState = BOOT;

  Serial.begin(9600);
  Serial.println("REBOOT");


  Wire.begin();
  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin,LOW);

 
  pinMode(ldr_pin, INPUT);
  myservo.attach(9);
  
}

void loop()
{
  a = height;
  
  EEPROM.write(addr, a);

  Serial.println("HEYY");

  //Serial.println(readEEPROM(eeprom, address), DEC);
  //address+=4;

  byte value = EEPROM.read(addr);
  Serial.print(addr);
  Serial.print("\t");
  Serial.print(value, DEC);
  Serial.println();
  addr = addr + 1;
  if (addr == EEPROM.length()) {
    addr = 0;
  }


  Serial.print("relative altitude: ");
  float height = getBMPAltitute();
  if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
  Serial.print(height, 1);
  Serial.print(" meters, ");

  setSoftwareState();
  delay(1000);
}




//As code begins,                         : Software state = 1 = BOOT;
//When prev_Alt > 5m and currAlt > prev_Alt : Software state = 2 = ACCENT
//When presAlt >670 and currAlt < prev_Alt : Software state = 3 = DEPLOYMENT (from rocket)
//When presAlt <450 and currAlt < prev_Alt : Software state = 4 = DESCENT (seperated from Container)
//When presAlt <5 and   currAlt < prev_Alt : Software state = 5 = END


           // Final code has been written in test ;

void setSoftwareState() {

  switch(oldSoftState){
    case BOOT:  
          if(prev_Alt > 5){
             pinMode(LED_BUILTIN,HIGH);
             oldSoftState = ACCENT;
          break;
            }
    case ACCENT:
          if(prev_Alt> 650){
            pinMode(LED_BUILTIN,LOW);
            oldSoftState = DEPLOYMENT;
          break;
            } 
          
    case DEPLOYMENT:
          if(prev_Alt < 500) {
            oldSoftState = DESCENT;
          servo_cutting(oldSoftState);
            }         //Sending command to camera subsystem
          //sendCommandtoCamera(SWITCHONCAMERASERVO);      //2 is switch on camera and servo command
          break;
    case DESCENT:
          if(prev_Alt < 5) {
            oldSoftState= END;
          pinMode(buzzer_pin, OUTPUT);
    digitalWrite(buzzer_pin, HIGH);
          break;
            }
    default: break;
  
  }

  prev_Alt = a;
  Serial.println("PREVALT = " + String(prev_Alt));
}














//void setSoftwareState() {
//  if (prev_Alt > 5 && a > prev_Alt && software_state == 1) {
//    software_state = 2;
//  } else if (a > 670 && a < prev_Alt && software_state == 2 || analogRead(ldr_pin) > 400) {
//    software_state = 3;
//  } else if (a < 500 && a < prev_Alt && software_state == 3) {
//    servo_cutting(software_state);
//  } else if (a < 450 && a < prev_Alt && software_state == 3) {
//    servo_cutting(software_state);
//    servo_cutting(software_state);
//    software_state = 4;
//  } else if (a < 5 && a < prev_Alt && software_state == 4) {
//    software_state = 5;
//    //============= SET BUZZER LOGIC HIGH =========================
//    pinMode(buzzer_pin, OUTPUT);
//    digitalWrite(buzzer_pin, HIGH);
//  }
//
//  prev_Alt = a;
//  Serial.println("PREVALT = " + String(prev_Alt));
//}
//
//




void servo_cutting(int soft_state) {
  if (soft_state == 3) {
    myservo.write(180);
    delay(800);
    myservo.write(0);
    delay(800);
  }
}
void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data )
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();

  delay(5);
}
byte readEEPROM(int deviceaddress, unsigned int eeaddress )   //WHY READ EEPROM
{
  byte rdata = 0xFF;

  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();

  Wire.requestFrom(deviceaddress, 1);

  if (Wire.available()) rdata = Wire.read();

  return rdata;
}

void initBmp(){
  if (!bmp.begin()) {
    #ifdef SER_DEBUG
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    #endif

      return;
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */ 

               
}

void setGroundAltitude(){
  double sum = 0;
  for(int i=0; i<2000; i++){
    sum += bmp.readAltitude(1013.25);
  }
  ground_altitude = sum/2000;
}

void setGroundAltitude(float alt){
  ground_altitude = alt;
}

volatile float getBMPAltitute(){
  return (bmp.readAltitude(1013.25) - ground_altitude);
}
