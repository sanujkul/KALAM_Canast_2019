//This function should be called in setup()
//This will initialize BUZZERPIN (14) as OUTPUT
void buzzerPinInIt(){
  pinMode(BUZZERPIN, OUTPUT);
}

//To be called by loop function
//when software state is 5 in last stage
void buzzerBajaDo() {
  digitalWrite(BUZZERPIN,HIGH);
}

//For testing purposes
void buzzerRokDo(){
  digitalWrite(BUZZERPIN,LOW);
}
