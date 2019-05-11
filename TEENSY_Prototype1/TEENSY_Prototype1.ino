//IMPORTANT : MPU toc library change address to 0x69
            //AD0 pin high
#include "pindef.h"

packet dataPacket(TEAM_ID); 
uint64_t packetCount = 0;

//////////////////////////I2C Objects Initialization=================================
Adafruit_BMP280 bmp;    //1. BMP object: I2C interface //Used "Wire"
RTC_DS3231 rtc;         //2. RTC object: I2C interface //Used "Wire"
MPU6050 mpu6050(Wire);  //3. MPU6050 object

//////////////////////////GPS Object Initialization=================================
TinyGPSPlus gps;        //4. The TinyGPS++ object



//////////////////////////5. File Objects for SDCARD=================================
File packetFile;    //file handle for packet.csv
File missionLog;  //file handle for missin.log
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
  delay(2000);
  //Opening SErial Monitor
  Serial.begin(9600);
  while(!Serial);

  //I2C Devices Initialization======================================
  Wire.begin();         //I2C Wire initialization
  initBmp();            //BMP
  initRTC();            //RTC
  resetMissionTime();   //To initialize the startTime variable
  mpu6050.begin();      //MPU
  mpu6050.calcGyroOffsets(true);
  
  //UART Devices Initialization===============================
  xbee.begin(9600);       //XBEE initializing
  initXBee();
  
  
  //SPI Devices Initialization=====================================
  initSD();

  //Battery Voltage:
  initBatteryVoltage();

  //GPS Initialization=====================================
//  initGPS();
  gps_uart.begin(GPSBaud);
  setGPSValues();
  smartDelay(1000);

  //HALL=============================================================
  initHall();

  //SOFTWARE STATE=============================================================
  dataPacket.software_state = BOOT;
} 

void loop() {
  
  mpu6050.update();
  if(1){
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
    smartDelay(900);
//====================================SOFTWARE STATE====================================================
    setSoftwareState();
    //====================================BLADE_SPIN=====================================================
    dataPacket.blade_spin_rate = giveRPM(countStartTime);
    makeCountZero();
    countStartTime = millis()%10000;  //unit is in miliseconds
    //================================================================================================
    dataPacket.display();
    //Sending data via xbee
    savePacket(&dataPacket);
    transmitPacketString(&dataPacket);
    
//    timer = millis();
  }

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
void setSoftwareState(){
  if(prevAlt > 5 && dataPacket.altitude > prevAlt && dataPacket.software_state == BOOT){
    dataPacket.software_state = ACCENT; //(i.e. 2)
  }else if(dataPacket.altitude > 670 && dataPacket.altitude < prevAlt && dataPacket.software_state == ACCENT){
    dataPacket.software_state = DEPLOYMENT; //(i.e. 3)
  }else if(dataPacket.altitude < 450 && dataPacket.altitude < prevAlt && dataPacket.software_state == DEPLOYMENT){
    dataPacket.software_state = DESCENT; //(i.e. 4)
  }else if(dataPacket.altitude < 5 && dataPacket.altitude < prevAlt && dataPacket.software_state == DESCENT){
    dataPacket.software_state = END; //(i.e. 5)
  }
  prevAlt = dataPacket.altitude;
  Serial.println("PREVALT = "+String(prevAlt));
}
