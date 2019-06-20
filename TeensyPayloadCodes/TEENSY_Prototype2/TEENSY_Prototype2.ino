//IMPORTANT : MPU toc library change address to 0x69
            //AD0 pin high
#include "pindef.h"

packet dataPacket(TEAM_ID); 
int packetCount = 0;

//////////////////////////I2C Objects Initialization=================================
Adafruit_BMP280 bmp;    //1. BMP object: I2C interface //Used "Wire"
RTC_DS3231 rtc;         //2. RTC object: I2C interface //Used "Wire"
MPU6050 mpu6050(Wire);  //3. MPU6050 object

//////////////////////////GPS Object Initialization=================================
TinyGPSPlus gps;        //4. The TinyGPS++ object



//////////////////////////5. File Objects for SDCARD=================================
File packetFile;    //file handle for packet.csv
File missionLog;  //file handle for mission.log

File backup;  // File handle for backup.txt that will store callibrated values.
File backupPacketCount;
//////////////////////////6. XBEE=================================
   //NOthing comes here
//////////////////////////7. HALL SENSOR=================================   
  //NOthing comes here
//////////////////////////7. SOFTWARE STATE================================= 
float prevAlt = 0;

long timer = 0;
volatile unsigned long countStartTime = 0;

/////////////////////////////////________________________ SETUP ________________________/////////////////////////////////
void setup() {
  
#ifdef SER_DEBUG
  //Opening SErial Monitor
  delay(2000);
  Serial.begin(9600);
  while(!Serial);
#endif

//SPI Devices Initialization=====================================
  initSD();
  
  //===========>> BACKUP CODE //FOR BACKING UP MISSION TIME IN SD CARD
  int lastMissionTime = 0;
  bool backUpThere = doesBackUpExist();
  Serial.println("BACKUP? = "+String(backUpThere));
  initializeSDFiles(backUpThere);
  
  //I2C Devices Initialization======================================
  Wire.begin();         //I2C Wire initialization
  initRTC();            //RTC
  initBmp();            //BMP
  mpu6050.begin();      //MPU
  
  if(backUpThere){
    callibrateUsingPrevDatafromSD();
    setPacketCountFromSD();
  }else{  //if There is no backup file
    //RTC :
    resetMissionTime();   //To initialize the startTime variable
    setGroundAltitude(); 
    mpu6050.calcGyroOffsets(true);
    //SOFTWARE STATE==============================
    dataPacket.software_state = BOOT;

    //Save backup of this updated data
#ifdef SER_DEBUG
  Serial.println("Calling saveBackup function");
#endif  
    saveBackUp();
  }

  bool backUpPacketThere = doesBackUpPacketExist();
  if(backUpPacketThere){
    packetCount = setPacketCountFromSD();
  }else{
    Serial.println("packetCount = 0");
    packetCount = 0;
  }
  
  //UART Devices Initialization===============================
  xbee.begin(9600);       //XBEE initializing
  initXBee();
  
  //Battery Voltage:
  initBatteryVoltage();

  //GPS Initialization=====================================
//  initGPS();
  gps_uart.begin(GPSBaud);
  setGPSValues();
  smartDelay(1000);

  //HALL=============================================================
  initHall();
  //
  buzzerPinInIt();

  initBluetooth();


  
} 

//long loopStartTime = 0;
//long loopEndTime = 0;

void loop() {
//  loopStartTime = millis()/10000;
  mpu6050.update();
  
    packetCount++;
    //====================================PACKET_COUNT
    dataPacket.packet_count = packetCount;
    //====================================BMP=========================================================
    dataPacket.altitude = getBMPAltitute();
    dataPacket.pressure = bmp.readPressure();
    dataPacket.temperature = bmp.readTemperature();
    //====================================RTC=========================================================
    dataPacket.mission_time = getMissionTime();
    //====================================MPU=========================================================
    dataPacket.pitch = mpu6050.getAngleX();
    dataPacket.roll  = mpu6050.getAngleY();
    //====================================VOLTAGE========================================================================
    dataPacket.voltage = getBatteryVoltage();
    //====================================GPS====================================================
    setGPSValues();
    smartDelay(800);
//====================================SOFTWARE STATE====================================================
    setSoftwareState2();
    //====================================BLADE_SPIN=====================================================
    dataPacket.blade_spin_rate = giveRPM(countStartTime);
    makeCountZero();
    countStartTime = millis()%10000;  //unit is in miliseconds
    //================================================================================================
    dataPacket.bonus_direction = getCameraDirection();

#ifdef SER_DEBUG
    dataPacket.display();
#endif
    //Saving packet to sd card
    savePacket(&dataPacket);
    
    //Backing up PacketCount to SD card:
    savePacketCount();
    //Sending data via xbee
    transmitPacketString(&dataPacket);
    
    
//    Serial.println(dataPacket.bonus_direction)
  
  
}

void setGPSValues(){
  dataPacket.gps_time_utc = getGPSTime();
  dataPacket.gps_sats = getGPSNumSat();
  dataPacket.gps_lattitude = getGPSLat();
  dataPacket.gps_longitude = getGPSLong();
  dataPacket.gps_altitude = getGPSAlt();
}

//As code begins,                         : Software state = 1 = BOOT;
//When prevAlt > 5m and currAlt > prevAlt : Software state = 2 = ACCENT
//When presAlt >670 and currAlt < prevAlt : Software state = 3 = DEPLOYMENT (from rocket)
//When presAlt <450 and currAlt < prevAlt : Software state = 4 = DESCENT (seperated from Container)
//When presAlt <5 and   currAlt < prevAlt : Software state = 5 = END
//void setSoftwareState(){
//  if(prevAlt > 5 && dataPacket.altitude > prevAlt && dataPacket.software_state == BOOT){
//    dataPacket.software_state = ACCENT; //(i.e. 2)
//  }else if(dataPacket.altitude > 670 && dataPacket.altitude < prevAlt && dataPacket.software_state == ACCENT){
//    dataPacket.software_state = DEPLOYMENT; //(i.e. 3)
//  }else if(dataPacket.altitude < 450 /*&& dataPacket.altitude < prevAlt*/ && dataPacket.software_state == DEPLOYMENT){
//    dataPacket.software_state = DESCENT; //(i.e. 4)
//  }else if(dataPacket.altitude < 5 /*&& dataPacket.altitude < prevAlt*/ && dataPacket.software_state == DESCENT){
//    dataPacket.software_state = END; //(i.e. 5)
//    buzzerBajaDo();
//  }
//  prevAlt = dataPacket.altitude;
////  Serial.println("PREVALT = "+String(prevAlt));
//}

void setSoftwareState2(){
  int oldSoftState = dataPacket.software_state;
  switch(oldSoftState){
    case BOOT:  
          if(dataPacket.altitude > 5) dataPacket.software_state = ACCENT;
          break;
    case ACCENT:
          if(dataPacket.altitude > 650) dataPacket.software_state = DEPLOYMENT;
          break;
    case DEPLOYMENT:
          if(dataPacket.altitude < 500) dataPacket.software_state = DESCENT;
          //Sending command to camera subsystem
          sendCommandtoCamera(SWITCHONCAMERASERVO);      //2 is switch on camera and servo command
          break;
    case DESCENT:
          if(dataPacket.altitude < 5) dataPacket.software_state = END;
          buzzerBajaDo();
          break;
    default: break;
  }

  if(oldSoftState != dataPacket.software_state){
    backupUpdatedSoftwareState(dataPacket.software_state);
  }
//  prevAlt = dataPacket.altitude;
}
