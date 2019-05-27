#include "pindef.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

MPU6050 mpu;                                          //Making a MPU6050 object!
//MPU6050 mpu(0x69);                                  // <-- use for AD0 high

// Define the servo motors
Servo servo0;                     //SERVO ZERO IS FOR YAW:

void setup() {
//  initCamera();
  
  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(38400);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately


//  initStabilize();
   
}



void loop() {
  // put your main code here, to run repeatedly:
  stabilizeLoop();

  if(millis()>10000 && !isCameraOn())
    switchOnOffCamera();

  if(millis()>600000 && isCameraOn()){
    switchOnOffCamera();
}
