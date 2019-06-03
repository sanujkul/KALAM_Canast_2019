/*
 * How to configure and pair two HC-05 Bluetooth Modules
 * by Dejan Nedelkovski, www.HowToMechatronics.com
 * 
 *                 == MASTER CODE ==
 */
//#define ledPin 9
int cam;
String cam2;
//int potValue = 0;



void setup() {
//  pinMode(ledPin, OUTPUT);
//  digitalWrite(ledPin, LOW);
  Serial.begin(9600);
  Serial2.begin(9600); // Default communication rate of the Bluetooth module
//  Serial2.setTimeout(100);
}

void loop() {
  
 cam2 = "-1";
 Serial2.print("1");

 Serial.println(millis());
 cam2 = Serial2.readString();
 Serial.println(millis());
 Serial.println("====="+cam2+"=");
 

 delay(600);
}
