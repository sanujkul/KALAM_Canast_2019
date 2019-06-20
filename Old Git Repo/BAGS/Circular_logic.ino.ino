#include <AutoPID.h>
#include <Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

#define KP 1
#define KD 0
#define KI 0
#define MPU_READ_DELAY 200
#define OUTPUT_MAX 360
#define OUTPUT_MIN 0

Servo myservo;
Madgwick filter;
MPU6050 mpu6050(Wire);
float ax, ay, az;
float gx, gy, gz;
float roll, pitch;
double yaw_val, reference, servo_speed;
unsigned long lastMPUUpdate;
int temp;

AutoPID myPID(&yaw_val, &reference, &servo_speed, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  myservo.attach(9);
  filter.begin(50);
  update_yaw();
  reference = yaw_val;
  Serial.println("Reference" + String(reference, 4));
  Serial.println("");
  myPID.setTimeStep(100);
}

void loop() {
  if (abs(reference - yaw_val)<0.1) {
    enter_rotation();
  }
  update_yaw();
  myPID.run();
  float map_val = map(servo_speed, 0, 360, 90, 180);
  Serial.println("servo_speed" + String(servo_speed));
  Serial.println("Error" + String(yaw_val - reference, 4));
  Serial.println("Servo Speed" + String((map_val), 4));
  Serial.println("Reference Point" + String(reference, 4));
  Serial.println("-----------------------------------------");
  //myservo.write(int(map_val));
  circular_logic();
}

float speed(float current, float last_value, float time_stamps){
  return (((current-last_value)/time_stamps));
}
  
void update_yaw() {
  if ((micros() - lastMPUUpdate) > MPU_READ_DELAY) {
    mpu6050.update();
    ax = mpu6050.getAccX();
    ay = mpu6050.getAccY();
    az = mpu6050.getAccZ();
    gx = mpu6050.getGyroX();
    gy = mpu6050.getGyroY();
    gz = mpu6050.getGyroZ();
    filter.updateIMU(gx, gy, gz, ax, ay, az);
    yaw_val = filter.getYaw();
    lastMPUUpdate = micros();
  }
}

void enter_rotation() {
  int data;
  while(!Serial.available()) {
  }
  data = Serial.read();
  Serial.println(data);
  delay(1000);
  myservo.write(data);
}

void circular_logic(){
  if (yaw_val == 0 || yaw_val == 360) {
    myservo.write(90);
  }
  
  if(yaw_val > 0 && yaw_val <180){
    if (yaw_val <45){
      myservo.write(112.5);
    }
    else if(yaw_val > 45 && yaw_val < 90 ){
      myservo.write(135);
    }
    else if(yaw_val > 90 && yaw_val < 135){
      myservo.write(157.5);
    }
    else if (yaw_val > 135 && yaw_val < 180){
      myservo.write(180);
    }
  }
  else if(yaw_val > 180 && yaw_val < 360){
    if (yaw_val < 225) {
      myservo.write(0);
    }
    else if(yaw_val > 225 && yaw_val < 270 ){
      myservo.write(23);
    }
    else if(yaw_val > 270 && yaw_val < 315){
      myservo.write(45);
    }
    else if (yaw_val > 315 && yaw_val < 360){
      myservo.write(68);
    }
  }
}
