int startingPitch;
int startingRoll;

void setStartingPitch(){
  startingPitch = mpu6050.getAngleX();
}

void setStartingRoll(){
  startingRoll = mpu6050.getAngleY();
}

float getMPUPitch(){
  return mpu6050.getAngleX();
}

float getMPURoll(){
  return mpu6050.getAngleY();
}

int getRelativePitch(){
  return mpu6050.getAngleX() - startingPitch;
}

int getRelativeRoll(){
  return mpu6050.getAngleY() - startingRoll;
}
