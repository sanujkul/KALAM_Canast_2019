void initXBee(){
  //zigbee.begin(9600); //Already in setup() of TEENSY_PROTOTYPE1 
//   xbee.onReceive(onRecieveData);
  pinMode(XBEE_INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(XBEE_INTERRUPT_PIN), xbeeMessegeReceived, RISING);
}

//Function That will be executed by xbee interrupt
void xbeeMessegeReceived(){
  int x = xbee.parseInt();
  switch(x){
    case 0:Serial.println("Recieved 0");
           break;
    case 1:Serial.println("Recieved 1"); 
           break;
    case 9://Detach the interrupt and stop this ISR if user ground station sends 9
           detachInterrupt(digitalPinToInterrupt(XBEE_INTERRUPT_PIN));
           break; 
    default:Serial.println("Recieved something");
            break;
     //and so on
  }
}

void calibrateSensors(){
  
}
//
////For transmitting data, function is in i_packet.ino
