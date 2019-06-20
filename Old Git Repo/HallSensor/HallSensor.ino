#define NUMBEROFMAGNETS 3           //Number of magnets on fan 
int dPin = 2;   
volatile unsigned long count = 0;

//=====================>> SETUP <<======================================
void setup() {
//  Serial.begin(9600);
  pinMode(dPin, INPUT);
  count = 0;                        //Initializing count
}
//=====================>> SETUP ENDS <<======================================
//=====================>> LOOP <<======================================
void loop() {
  attachInterrupt(digitalPinToInterrupt(dPin), raiseCount, FALLING); //Attaching the interrupts
  volatile unsigned long interruptStartTime = millis()/1000;                   //Time (in second) when we started our interrupts
  /*
   * Some other Sensor codes here
   */
   
  long rpm = giveRPM(interruptStartTime);
  detachInterrupt(digitalPinToInterrupt(dPin));             //Detaching the interrupt
  count = 0;                                                //Reseting count to 0
  
}
//=====================>> LOOP ENDS <<=================================

/*
 * Called by Hardware Interrupt at digital pin
 * Function: Increase global variable count by 1
 */
int raiseCount(){                             
  count++;
}


/*
 * RPM = number of rotations/time taken * 60
 *     = (count2)/(currentTime - time1) * 60 .   (count is count per magnet)
 */
long giveRPM(volatile unsigned long interruptStartTime){
  volatile unsigned long timeTaken = ((millis()/1000) - interruptStartTime); 
  volatile unsigned long noOfRotations = count/NUMBEROFMAGNETS;    //Detection of "numberOfMagnets" of counts equals 1 rotation in that time
  float rpm = (noOfRotations/timeTaken)*60;                       //Maths to get RPM
  return rpm;
}

