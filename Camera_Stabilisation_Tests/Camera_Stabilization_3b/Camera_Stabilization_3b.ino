/*
  DIY Gimbal - MPU6050 Arduino Tutorial
  by Dejan, www.HowToMechatronics.com
  Code based on the MPU6050_DMP6 example from the i2cdevlib library by Jeff Rowberg:
  https://github.com/jrowberg/i2cdevlib
*/
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#include <Servo.h>
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;                                          //Making a MPU6050 object!
//MPU6050 mpu(0x69);                                  // <-- use for AD0 high

// Define the 3 servo motors
Servo servo0;                     //SERVO ZERO IS FOR YAW:
int servo0Speed = 1500;
//Servo servo1;
//Servo servo2;

float yaw0 = 0;
float correct;
int j = 0;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===               MY CODE FOR CALCULATING GYRO ERRORS        ===
// ===              CODE TAKEN FROM HOW TO MECHTRONICS          ===
// ================================================================
  const int MPUAddr = 0x68;
  float AccX, AccY, AccZ;
  float GyroX, GyroY, GyroZ;
  float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
  float roll, pitch, yaw;
  float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
  float elapsedTime, currentTime, previousTime;
  int c = 0;

  void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
    while (c < 200) {
      Wire.beginTransmission(MPUAddr);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUAddr, 6, true);
      AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
      AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
      AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
      // Sum all readings
      AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
      AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
      c++;
    }
    //Divide the sum by 200 to get the error value
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    c = 0;
    // Read gyro values 200 times
    while (c < 200) {
      Wire.beginTransmission(MPUAddr);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPUAddr, 6, true);
      GyroX = Wire.read() << 8 | Wire.read();
      GyroY = Wire.read() << 8 | Wire.read();
      GyroZ = Wire.read() << 8 | Wire.read();
      // Sum all readings
      GyroErrorX = GyroErrorX + (GyroX / 131.0);
      GyroErrorY = GyroErrorY + (GyroY / 131.0);
      GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
      c++;
    }
    //Divide the sum by 200 to get the error value
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

////////////////////////////////////////////////////////////////////
//         CALVULATE YAW

//call repeatedly in loop, only updates after a certain time interval
//returns true if update happened
////////////////////////////////////////////////////////////////////
void calculateYaw() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Get Yaw, Pitch and Roll values
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Yaw, Pitch, Roll values - Radians to degrees
    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
#endif
  }
}//void calculateYaw

// ================================================================
// ===                      CAMERA RELATED                       ===
// ================================================================
#define LONG_PRESS 3000
#define PIN_CAMERA 4
long cameraStartime = 0;
int cameraState = 0;

void setupCamera(){
  pinMode(PIN_CAMERA,OUTPUT);
  cameraStartime = millis();
}

void switchOnOffCamera(){
  digitalWrite(PIN_CAMERA,HIGH);
  Serial.write("LP");
  delay(LONG_PRESS);
  digitalWrite(PIN_CAMERA,LOW);
  cameraState = 1;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

//  setupCamera();
//  switchOnOffCamera();
//  cameraState = 1;  //.i.e. ON
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(38400);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

//   ================================================================
// ===              InSETUP:  MY CODE FOR CALCULATING GYRO ERRORS        ===
// ===              In SETUP:CODE TAKEN FROM HOW TO MECHTRONICS          ===
// ================================================================
  Wire.beginTransmission(MPUAddr);       // Start communication with MPUAddr6050 // MPUAddr=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
// ================================================================
   //End of my code:
// ================================================================


    
  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  //Commenting there's
//  mpu.setXGyroOffset(17);
//  mpu.setYGyroOffset(-69);
//  mpu.setZGyroOffset(27);
//  mpu.setZAccelOffset(1551); // 1688 factory default for my test chip

  //These are mine:
  mpu.setXGyroOffset(GyroErrorX);
  mpu.setYGyroOffset(GyroErrorY);
  mpu.setZGyroOffset(GyroErrorZ);
  mpu.setZAccelOffset(725); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    // Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }

  // Define the pins to which the 3 servo motors are connected
  servo0.attach(10);
//  servo1.attach(9);
//  servo2.attach(8);
  Serial.println("SETUP FINISHED");

  //SELF CALLIBRATION:
  Serial.println("SELF CALLIBRATION STARTED");
  //For 2000 times, we'll record yaw values
  for(int i=0; i<2000; i++){
    calculateYaw();
    correct = ypr[0];
  }
  //This will indicate that callibration is finished:
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  
}

// ================================================================
// ===                     LOOP                       ===
// ================================================================

void loop() {
  calculateYaw();
  ypr[0] = ypr[0] - correct;

  Serial.print(String(ypr[0]) + "\t");
  
  //TO KEEP RANGE OF YAW FROM -180 to 180
  yaw0 = ypr[0];
  if(yaw0 > 180){
    yaw0 = 360 - ypr[0];
  }else if(yaw0 < -180){
    yaw0 = 360 + ypr[0];
  }

  Serial.print(String(yaw0));  
  rotateServo();  

  //Switching off the camera after 1 minute
//  if((millis() - cameraStartime) > 120000 && cameraState == 1){
//    switchOnOffCamera();  
//    digitalWrite(13,LOW); 
//    cameraState = 0;
//  }
}

void rotateServo(){
  if(yaw0 > 5){
    servo0Speed = map(yaw0, 0, 110, 1425, 1395);
    if(yaw0 > 70) 
        servo0Speed= 1000;
    rotateServoClockwise(servo0Speed);
  }else if(yaw0 < -5){
    servo0Speed = map(yaw0, -110, 0, 1585, 1550);
    if(yaw0 < -70) 
        servo0Speed= 2000;
    rotateServoAntiClockwise(servo0Speed);
  }else{
    servo0Speed = 1500;
    stopServo();
  }
  Serial.print("    ");
  Serial.println(servo0Speed);
}

void rotateServo2(){
  if(ypr[0] > 10){
    servo0Speed = map(ypr[0], 0, 180, 1425, 1395);
    rotateServoClockwise(servo0Speed);
  }else if(ypr[0] < -10){
    servo0Speed = map(ypr[0], -180, 0, 1580, 1550);
    rotateServoAntiClockwise(servo0Speed);
  }else{
    servo0Speed = 1500;
    stopServo();
  }
  
}

//Continuous servo rotates clockwise for Servo.write() value between 0 to 89;
void rotateServoClockwise(int servoSpeed){
  servo0.writeMicroseconds(servoSpeed);
}

//Continuous servo rotates clockwise for Servo.write() value between 180 down to 91;
void rotateServoAntiClockwise(int servoSpeed){
  servo0.writeMicroseconds(servoSpeed);
}

void stopServo(){
  servo0.writeMicroseconds(1500);
}
