#include <Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

Servo myservo;
Madgwick filter;
MPU6050 mpu6050(Wire);
int servo_speed = 90;// Variable to store stepper speed levels
float payload_speed;//Variable to store gyro speed levels between -83 and 83
float  yaw, prev_yaw;
unsigned long microsPerReading, microsPrevious;
bool clockwise = 0;
//int val;    // variable to read the value from the analog pin

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("Begin");
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  filter.begin(50);

  microsPerReading = 1000000 / 50;
  microsPrevious = micros();
  /* logic to store 8 values about 90 */
}

void loop() {

  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  microsNow = micros();
 if (microsNow - microsPrevious >= microsPerReading) {
    mpu6050.update();
    ax = mpu6050.getAccX();
    ay = mpu6050.getAccY();
    az = mpu6050.getAccZ();
    gx = mpu6050.getGyroX();
    gy = mpu6050.getGyroY();
    gz = mpu6050.getGyroZ();

    filter.updateIMU(gx, gy, gz, ax, ay, az);
    yaw = filter.getYaw();
    //Serial.println(yaw);
    if(yaw < prev_yaw){
      clockwise = true;
    }
    else{
      clockwise = false;
    }
    //payload_speed = speed(yaw,prev_yaw,microsPerReading);
    payload_speed = yaw;
    Serial.println(payload_speed);
    //Serial.println(clockwise);
    if (int(yaw) == int(prev_yaw)){
      servo_speed = 90;
      //Serial.println("in stop");
    }
    else{
      if (clockwise && payload_speed < 8.68) {
        servo_speed = map(payload_speed, -8.68, 0, 180, 90);

      }
      else if (!clockwise && payload_speed < 8.68) {
        servo_speed = map(payload_speed, 0, 8.68, 90, 0);
      }
      else if (payload_speed > 8.68){
        if (!clockwise) {
          servo_speed = 0;
        }
        else {
          servo_speed = 180;
        }
      }
    }

      //Serial.println(servo_speed);
      //Serial.println(payload_speed);
      myservo.write(servo_speed);
      prev_yaw = yaw;
     microsPrevious = microsNow; 
    }
}
    /*while(Serial.available() == 0);
      int val = Serial.parseInt();
      Serial.println((byte)val);
      myservo.write((byte)val);                  // sets the servo position according to the scaled value*/
float speed(float current, float last_value, float time_stamps){
  return (((current-last_value)/time_stamps));
}
