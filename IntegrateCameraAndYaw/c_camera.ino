// ================================================================
// ===                      CAMERA RELATED                       ===
// ================================================================


long cameraStartime = 0;
int cameraState = 0;

void initCamera(){
  pinMode(PIN_CAMERA,OUTPUT);
  digitalWrite(PIN_CAMERA,HIGH);
  cameraState = 0;       //i.e. OFF
}

void switchOnOffCamera(){
  digitalWrite(PIN_CAMERA,LOW);
  delay(LONG_PRESS);
  digitalWrite(PIN_CAMERA,HIGH);
  cameraState = (cameraState == 0)?1:0;
}

boolean isCameraOn(){
  if(cameraState == 1){
    return true;
  }else{
    return false;
  }
}
