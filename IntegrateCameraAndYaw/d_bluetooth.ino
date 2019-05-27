int BTcommand;

void initBluetooth(){
  Serial.begin(9600);
  Serial.setTimeout(10); //10 milliseconds
}

int getCommand(){
  BTcommand = -1;
  BTcommand = Serial.parseInt();
#ifdef SER_DEBUG
  Serial.println("BTCOMMAND : "+String(BTcommand));  
#endif
  
  return BTcommand; 
}
