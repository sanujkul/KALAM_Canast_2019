volatile unsigned long count = 0;

float rpm[3];
int position_i;

void initHall() {
  pinMode(HALLPIN, INPUT);
  count = 0;                        //Initializing count
  attachInterrupt(digitalPinToInterrupt(HALLPIN), raiseCount, CHANGE);  //HENCE increase of 2 when detected
  position_i = 0;
  
  for(int i =0; i<3; i++){
    rpm[i] = 0;
  }
  
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
  float instantaneous_rpm = ((float)noOfRotations/(timeTaken))*60*1000;                       //Maths to get RPM

  rpm[position_i] = instantaneous_rpm;
  position_i = (position_i+1)%3;
  
#ifdef SER_DEBUG
  Serial.print("inst RPM = "+String(instantaneous_rpm)+"\t POS = "+String(position_i));
#endif
  
  return findAverageRPM();
  
}

void makeCountZero(){
  count = 0;
}

long findAverageRPM(){
//  Serial.print("========>> IN findAVgRPM");
//  Serial.println(rpm[0]+rpm[1]+rpm[2]);
  return (long)(((rpm[0]+rpm[1]+rpm[2])/3)*3.14/30);  //converting rmp to radians per 
}
