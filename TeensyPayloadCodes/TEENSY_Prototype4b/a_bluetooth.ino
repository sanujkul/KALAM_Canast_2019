String cam;

void initBluetooth() {
   Serial2.begin(9600); //communication rate of the Bluetooth module
   cam = -1;
   Serial2.setTimeout(100);
}

String getCameraDirection() {
  cam = "-1";
  Serial2.print("1");
  cam = Serial2.readString();
  if(cam.length() == 0){
    return "-1";
  }

  int dir = cam.toInt();
  dir = dir/2;
  return String(dir);
  
}

void sendCommandtoCamera(int com){
  Serial2.print(String(com));
}
