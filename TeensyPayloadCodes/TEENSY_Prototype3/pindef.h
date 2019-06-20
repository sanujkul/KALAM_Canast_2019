#include <MPU6050_tockn.h>
#include <RTClib.h>    //RTC Library
#include <Arduino.h>   // required before wiring_private.h
#include <Wire.h>                            
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>

//BMP: I2C pins : SCL-19, SDA-18 and Vin = 3.3V, SDO - HIGH (SHORT TO Vin)
//SPI : SS - 10, SCK -13, MOSI-11, MISO-12

#include <Adafruit_BMP280.h> //BMP Library
//#include "MPU6050_tockn.h"
#include "packet.h"

#define voltagePin A8

#define gps_uart Serial3
#define GPSBaud 9600

#define xbee Serial1
#define XBEEBaud 9600
#define XBEE_INTERRUPT_PIN 2

#define SD_SELECT 6

#define NUMBEROFMAGNETS 1           //Number of magnets on fan 
#define HALLPIN 5

#define BUZZERPIN 14

#define LOG_MISSION	//enable this to log mission events to SD card
#define SER_DEBUG		//enable this to get debug info in serial monitor

//#undef SER_DEBUG


#define SWITCHONCAMERASERVO 2
#define SWITCHONCAMERA 3

#define TEAM_ID 3279		//TEAM KALAM
