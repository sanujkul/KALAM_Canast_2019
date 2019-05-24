int cam = -1;

void initBluetooth() {
   Serial2.begin(9600); //communication rate of the Bluetooth module
   cam = 0;
}

int getCameraDirection() {
  cam = -1;
  long BTTimeStart = millis();
  Serial2.print("1\n");
  while(!Serial2.available()){ // Checks whether data is comming from the Serial2 port
//    Serial.println("Recieve Some data");
     // Reads the data from the Serial2 port
     if(millis() - BTTimeStart > 50){
      break;
     }
  }
  if(Serial2.available()){
    cam = Serial2.parseInt();
 }
 return cam;
}
