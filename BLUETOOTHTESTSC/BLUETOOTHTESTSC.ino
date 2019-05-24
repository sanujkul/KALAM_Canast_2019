#include <MPU6050_tockn.h>
#include <Wire.h>
int count = 0;
MPU6050 mpu6050(Wire);
int state = 0;
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
 // mpu6050.calcGyroOffsets(true);
 inputString.reserve(200);
  
}

void loop() {
  //mpu6050.update();
//  Serial.print("angleX : ");
//  Serial.print(mpu6050.getAngleX());
//  Serial.print("\tangleY : ");
//  Serial.print(mpu6050.getAngleY());
//  Serial.print("\tangleZ : ");
//  Serial.println(mpu6050.getAngleZ());
  state = -1;
    if(stringComplete){
      if(inputString.equals("1\n")){
        count++;
        Serial.print(String(count));
      }

      inputString = "";
      stringComplete = false;
    }
       
      //delay(200);
}
void serialEvent(){
  while(Serial.available()){
    char inChar = (char)Serial.read();
      inputString += inChar;
      if(inChar == '\n'){
        stringComplete = true; 
      }
    }
     
    
}
