/*
 * How to configure and pair two HC-05 Bluetooth Modules
 * by Dejan Nedelkovski, www.HowToMechatronics.com
 * 
 *                 == MASTER CODE ==
 */
//#define ledPin 9
int cam;
//int potValue = 0;



void setup() {
//  pinMode(ledPin, OUTPUT);
//  digitalWrite(ledPin, LOW);
  Serial.begin(9600);
  Serial2.begin(9600); // Default communication rate of the Bluetooth module
}

void loop() {


  
 
 cam = -1;
 long BTTimeStart = millis();
 Serial2.print("1\n");
 while(!Serial2.available()){ // Checks whether data is comming from the Serial2 port
//    Serial.println("Recieve Some data");
     // Reads the data from the Serial2 port
     if(millis() - BTTimeStart > 50){
      break;
     }
 }
 if(Serial2.available()){
    cam = Serial2.parseInt();
 }
 Serial.println(cam);
 
 // Controlling the LED
// if (state == '1') {
//  digitalWrite(ledPin, HIGH); // LED ON
//  state = 0;
// }
// else if (state == '0') {
//  digitalWrite(ledPin, LOW); // LED ON
//  state = 0;
// }
// // Reading the potentiometer
// potValue = analogRead(A0);
// int potValueMapped = map(potValue, 0, 1023, 0, 255);
// Serial2.write(potValueMapped); // Sends potValue to servo motor

 delay(600);
}
