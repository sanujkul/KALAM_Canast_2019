# KALAM_Canast_2019    
These codes/files were written/developed for the Cansat competition 2019, USA.    

This folder have two types of files:  
1. Codes (.c, .c++, .ino)    
2. PCB designs    
  
Cansat 2019 was divided ino two parts: A payload and a container. Cansat is launched to the height of 750m and at the height of 450m, during descent journey,container would release the payload which will descent via auto-gyro mechanism.    
Cansat has to perform telemetry throughout its journey.   
  
Files descriptions:  
a. TeensyPayloadCodes : Have the Cansat's payload codes.   
b. Container_Final : Have the Cansat's container codes.    
c. Camera Stabilization Codes: Payload was divided into 2 subparts, minor subpart was camra stabilization system and these are codes for stabilizing the camera to shoot in a particular direction.   
d. Camera_Stabilisation_Tests: Codes written for testing Camera stabilization system.  
e. Bluetooth Tests: Camera stabilization setup used to communicate to main microcontroller (Teensy) via bluetooth and these are test codes for that.   
f. SumiranPCB: Contains PCB designs (Eagle Cad files) for the cansat.  
g. Packets: This is a class developed to use telemetry packet as an object to ease or work in organising the codeflow.  
