#include "pindef.h"
int command;
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

MPU6050 mpu;                                          //Making a MPU6050 object!
//MPU6050 mpu(0x69);                                  // <-- use for AD0 high

// Define the servo motors
Servo servo0;                     //SERVO ZERO IS FOR YAW:

void setup() {
  initCamera();
  initBluetooth();
  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
//  Serial.begin(38400);
//  while (!Serial); // wait for Leonardo enumeration, others continue immediately


  initStabilize();
   
}

//int count == 1

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println(m/illis());
  stabilizeLoop();

//  Serial.println(mill/is());

//Getting command from bluetooth
  command = getCommand();

#ifdef SER_DEBUG
  Serial.println("COMMAND : "+String(command));  
#endif

  switch(command){
    case 1: Serial.print(getYaw()); 
            break; 

    case 2: //Start servo rotation and switch on camera
            setStartServoRotation(true);     
            makeCameraPinLow();  
            break; 
    default:break;
  }

  if(shallWeTurnCameraPinHighAgain()){
    makeCameraPinHigh();
  }
  
}
