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
    case 1:Serial.println("Recieved 1: Callibration Command");
           calibrateSensors1(); 
           break;
    case 2:Serial.println("Recieved 2: Buzzer Baja Do");
           recieve2();
           break;
    case 3:Serial.println("Recieved 3: Buzzer Rok Do");
           recieve3();
           break;
    case 9://Detach the interrupt and stop this ISR if user ground station sends 9
           detachInterrupt(digitalPinToInterrupt(XBEE_INTERRUPT_PIN));
           break; 
    default:Serial.println("Recieved something");
            break;
     //and so on
  }
}

void calibrateSensors1(){
  //detachInterrupt so that while callibrating nothing disturbs it
  detachInterrupt(digitalPinToInterrupt(XBEE_INTERRUPT_PIN));
  unsigned long startTime = millis();
  //Now performing callibration commands: And suppose it will take 15 seconds
  for(int i=0;i<2;i++){
    Serial.println("Callibrating starts in: "+String(2-(i+1))+"second");
    delay(1000);
  }

  setGroundAltitude();  //FOR BMP
  mpu6050.calcGyroOffsets(true); //FOR MPU
  
  Serial.println("Callibration Finished");
  // Callibration finished:

  //Let attachInterrupt again in case we need to recieve some more commands
  attachInterrupt(digitalPinToInterrupt(XBEE_INTERRUPT_PIN), xbeeMessegeReceived, RISING);
}

void recieve2(){
  detachInterrupt(digitalPinToInterrupt(XBEE_INTERRUPT_PIN));
  for(int i=0;i<2;i++){
    Serial.println("Buzzer baja starts in: "+String(2-(i+1))+"second");
    delay(1000);
  }
  buzzerBajaDo();
//  Serial.println(BUZZERPIN);
//  digitalWrite(BUZZERPIN,HIGH);
  attachInterrupt(digitalPinToInterrupt(XBEE_INTERRUPT_PIN), xbeeMessegeReceived, RISING);
}

void recieve3(){
  detachInterrupt(digitalPinToInterrupt(XBEE_INTERRUPT_PIN));
  for(int i=0;i<2;i++){
    Serial.println("Buzzer stops in: "+String(2-(i+1))+"second");
    delay(1000);
  }
  buzzerRokDo();
//  digitalWrite(BUZZERPIN,LOW);
  attachInterrupt(digitalPinToInterrupt(XBEE_INTERRUPT_PIN), xbeeMessegeReceived, RISING);
}

//
////For transmitting data, function is in i_packet.ino
