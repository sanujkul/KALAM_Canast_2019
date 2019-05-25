
//FOR NMOS: 
//Gate --- > PULLED UP with 33k
//Drain connected to Vcc (CAMERA INPUT) with 1k
//Drain will be output 

int dPin = 4;
void setup() {
    Serial.begin(9600);
    pinMode(dPin, OUTPUT);
    digitalWrite(dPin,HIGH);
    delay(2000);
    Serial.println("SETUP FINISHED");
}

void loop() {
  //SWITCH ON/OFF
  Serial.println("ON");
    digitalWrite(13,HIGH);
    digitalWrite(dPin,LOW);
    delay(3000);
    digitalWrite(dPin,HIGH);

    delay(20000);

  //SWITCH OFF;
  Serial.println("OFF");
  digitalWrite(13,HIGH);
    digitalWrite(dPin,LOW);
    delay(3000);
    digitalWrite(dPin,HIGH);

//  digitalWrite(dPin,LOW);
  
  while(1){
    //
  }
    
}
