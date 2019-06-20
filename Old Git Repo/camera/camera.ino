#define LONG_PRESS 2000 // ms
#define LONG_PRESS 1000 // ms

void setup() {
  pinMode(BUTTON, OUTPUT);
}

void loop() {
  
}

void pressCamButton(byte button, long duration, byte times){
  for(int i=0; i<times; i++){
    digitalWrite(button, HIGH);
    delay(duration);
    digitalWrite(button, LOW);
  }
}

