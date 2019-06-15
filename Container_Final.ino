#include <Wire.h>
#include <SFE_BMP180.h>
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


 int oldSoftState = BOOT;
const int ldr_pin = A0;
const int buzzer_pin = A1;
SFE_BMP180 bmp180;
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

void setup()
{
  int oldSoftState = BOOT;

  Serial.begin(9600);
  Serial.println("REBOOT");


  Wire.begin();
  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin,LOW);

 
  pinMode(ldr_pin, INPUT);
  myservo.attach(9);
  // Initialize the sensor
  if (bmp180.begin())

  {
    Serial.println("BMP180 init success");
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_BUILTIN, HIGH);

    delay(5000);

    digitalWrite(LED_BUILTIN, LOW);
  }
  else
  {
    Serial.println("BMP180 init fail (disconnected?)\n\n");


  }

  // Get the baseline pressure:

  baseline = getPressure();   //  OBJECT REFEWRENCE MISSING


  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb");


}

void loop()
{
  double P;
  P = getPressure();

  // relative altitude difference between the new reading and the baseline reading:

  a = bmp180.altitude(P, baseline);
  //writeEEPROM(eeprom, address, a);
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
  if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
  Serial.print(a, 1);
  Serial.print(" meters, ");

  setSoftwareState();
  delay(1000);
}
double getPressure()
{
  char status;
  double T, P, p0, a;

  //  temperature measurement to perform a pressure reading.


  status = bmp180.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);  //WHY ??

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = bmp180.getTemperature(T);
    if (status != 0)
    {
      //  pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).

      status = bmp180.startPressure(3);
      if (status != 0)
      {
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.

        status = bmp180.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}



//As code begins,                         : Software state = 1 = BOOT;
//When prev_Alt > 5m and currAlt > prev_Alt : Software state = 2 = ACCENT
//When presAlt >670 and currAlt < prev_Alt : Software state = 3 = DEPLOYMENT (from rocket)
//When presAlt <450 and currAlt < prev_Alt : Software state = 4 = DESCENT (seperated from Container)
//When presAlt <5 and   currAlt < prev_Alt : Software state = 5 = END


           // Final code has been written in test ;

void setSoftwareState() {
//  if (prev_Alt > 5 && a > prev_Alt && software_state == 1) {
//    software_state = 2;
//    digitalWrite(LED_BUILTIN, LOW);
//  } else if (a > 8 && a < prev_Alt && software_state == 2 || analogRead(ldr_pin) > 400) {
//    software_state = 3;
//    digitalWrite(LED_BUILTIN, HIGH);
//  } else if (a < 9 && a < prev_Alt && software_state == 3) {
//    servo_cutting(software_state);
//    digitalWrite(LED_BUILTIN, LOW);
//  }
//  //  else if(a < 5 && a < prev_Alt && software_state == 3){
//  //    servo_cutting(software_state);
//  //    servo_cutting(software_state);
//  //     digitalWrite(LED_BUILTIN, HIGH);
//  //    software_state = 4;
//  //  }
//  else if (a < 5 && a < prev_Alt && software_state == 3) {
//    software_state = 5;
//    //============= SET BUZZER LOGIC HIGH =========================
//    pinMode(buzzer_pin, OUTPUT);
//    digitalWrite(buzzer_pin, HIGH);
//    digitalWrite(LED_BUILTIN, HIGH);

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
