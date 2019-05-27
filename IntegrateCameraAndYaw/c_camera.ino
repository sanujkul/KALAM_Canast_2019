// ================================================================
// ===                      CAMERA RELATED                       ===
// ================================================================
//long cameraStartime = 0;
//int cameraState = 0;
int cameraPinStatus = 0;
uint64_t cameraPinLowTime = 0;

void initCamera(){
  pinMode(PIN_CAMERA,OUTPUT);
  digitalWrite(PIN_CAMERA,HIGH);
  cameraPinStatus = 0;   //Camera Pin is HIGH
//  cameraState = 0;       //i.e. OFF
}

//void switchOnOffCamera(){
//  digitalWrite(PIN_CAMERA,LOW);
//  delay(LONG_PRESS);
//  digitalWrite(PIN_CAMERA,HIGH);
//  cameraState = (cameraState == 0)?1:0;
//}

void makeCameraPinLow(){
  digitalWrite(PIN_CAMERA,LOW);
  cameraPinStatus = 1;
  cameraPinLowTime = millis();
}

void makeCameraPinHigh(){
  digitalWrite(PIN_CAMERA,HIGH);
  cameraPinStatus = 0;
}

bool shallWeTurnCameraPinHighAgain(){
  uint64_t timeCheck = millis() - cameraPinLowTime;
  if(isCameraPinLow() && timeCheck > LONG_PRESS){
    return true;
  }else{
    return false;
  }
}

//bool isCameraOn(){
//  return (cameraState == 1);
//}

bool isCameraPinLow(){
  return (cameraPinStatus == 1);
}
