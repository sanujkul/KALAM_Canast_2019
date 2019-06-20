void initXBee(){
  //zigbee.begin(9600); //Already in setup() of cansat_ 
   zigbee.onReceive(onRecieveData);
}

void onRecieveData(){
  int i = zigbee.read();
  while(zigbee.available()){
    zigbee.read();
  }
  switch(i){
    case '0'://do this
           break;
    case '1': //do that        
            break;
     //and so on
  }
}

//For transmitting data, function is in i_packet.ino
