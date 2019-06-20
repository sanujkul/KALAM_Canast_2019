volatile unsigned long count = 0;

void initHall() {
  pinMode(HALLPIN, INPUT);
  count = 0;                        //Initializing count
  attachInterrupt(digitalPinToInterrupt(HALLPIN), raiseCount, CHANGE);  //HENCE increase of 2 when detected
}

void raiseCount(){                             
  count++;
}

long giveRPM(volatile unsigned long interruptStartTime){
  volatile unsigned long timeTaken = ((millis() - interruptStartTime))%10000;   //unit is in milliseconds 
  if(timeTaken < 0) {
    timeTaken = 10000 + timeTaken;
  }
  volatile unsigned long noOfRotations = (count/NUMBEROFMAGNETS)/2;    //Detection of "numberOfMagnets" of counts equals 1 rotation in that time
  float rpm = ((float)noOfRotations/(timeTaken))*60*1000;                       //Maths to get RPM
  Serial.print("RPM = "+String(rpm)+"\t COUNT = "+String(count));
  return rpm;
}

void makeCountZero(){
  count = 0;
}

