String packetFileName = "packets";
String packetFileExt  = "csv";
String completePacketFileName;

String missionFileName = "mission";
String missionFileExt = "log";
String completeMissionFileName;

String backupFileName = "backup.txt";
String backupPacketName = "count.txt";

String timeStamp = "2013-10-22T01:37:56+05:30";  //UTC Format: 1994-11-05T08:15:30-05:00 corresponds to November 5, 1994, 8:15:30 am, US Eastern Standard Time.

 /*
  Sets up the SD card and creates mission.log and packets.csv
  */
void initSD(){ 
  //----------Setting up sd card-----------//
#ifdef SER_DEBUG
    Serial.print("Initializing SD card...");
#endif  
  
  pinMode(SD_SELECT, OUTPUT);
  
  if (!SD.begin(SD_SELECT)) {
#ifdef SER_DEBUG
    Serial.println("initialization failed!");
#endif
    // don't do anything more:
//    while (1);
  }

  Serial.println("initialization done.");
  xbee.println("initialization done.");
  
  //initializing packetFile and missionLog objects declared in TEENSY_prototype
  //Instead of following two lines, calling the function initialize files by setup() 
//  packetFile  = newFile(packetFileName, packetFileExt,true);
//  missionLog  = newFile(missionFileName, missionFileExt,true); 
  
}

void initializeSDFiles(bool yesNo){
  packetFile  = newFile(packetFileName, packetFileExt,!yesNo);    //Third field true will delete existing
  missionLog  = newFile(missionFileName, missionFileExt,!yesNo);  //files of these names

}
 /*
  Creates and sets up a new file.
  */
  
File newFile(String fileName, String fileExt, bool delOldFile){
  File tempFile;
 //completeFileName = String(fileName + "_" + String(millis()) + "." + fileExt);
  String completeFileName_String = String(fileName + "." + fileExt); //filename cannot be too long

  int len = completeFileName_String.length();
  char completeFileName[len];
  completeFileName_String.toCharArray(completeFileName, len);
  
//  delete any pre-existing record
  if(SD.exists(completeFileName) && delOldFile){
    #ifdef SER_DEBUG
      Serial.println("Found " + completeFileName_String +". Removing it.");
    #endif
    SD.remove(completeFileName);
  }
  
  //Create a new file
  #ifdef SER_DEBUG
    Serial.println("Creating " + completeFileName_String + " ...");
  #endif
  
  if (tempFile = SD.open(completeFileName, FILE_WRITE)) {
    #ifdef SER_DEBUG
      Serial.println(completeFileName_String + " created.");
    #endif
  }
  else {  //die as new file could not be created
    #ifdef SER_DEBUG
      Serial.println("Failed to create " + completeFileName_String);
    #endif
    return *(new File());  //return an empty file object
  }
  tempFile.close();   //close the file
  return (tempFile);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//          LogEvent                              //////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


boolean logEvent(const String text){
  /*
  Safely stores the text to mission.log with time stamp
  */
  return safe_println(&missionLog, String(getTimeStamp() + "\t" + text));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//          Functions for STORING To SD CARD                              //////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

boolean safe_print(File* file_ptr, const String text){                                //FOR printing string to SD card
  /*
    Ensures no two files are opened at the same time, and data is saved to SD card
    Doesn't implement file locking, or stop interrupts.
  */
  File temp;
  //open file in write mode.
  if (!(temp = SD.open(file_ptr->name(), FILE_WRITE))) {
    #ifdef SER_DEBUG    //die
      Serial.println("\t Failed to open " + String(file_ptr->name()));
    #endif
    return false;
  }
  temp.print(text);
  #ifdef SER_DEBUG
    Serial.println("\t Printed the line: " + text);
  #endif
  temp.close(); 
  return true;
}

boolean safe_println(File* file_ptr, const String text){              //FOR printing one line to SD card so that new line starts at fresh line
  return safe_print(file_ptr, String(text + "\n"));
}

boolean safe_print(const String filename_String, const String text){
  int len = filename_String.length();
  char filename[len];
  filename_String.toCharArray(filename, len);
  
  File file =  SD.open(filename, FILE_READ);
  return safe_print(&file, text);
}

boolean safe_println(const String filename_String, const String text){
  int len = filename_String.length();
  char filename[len];
  filename_String.toCharArray(filename, len);
  
  File file =  SD.open(filename, FILE_READ);
  return safe_println(&file, text);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//          Functions for READING file                              //////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readFile(File* file_ptr){
  
  File temp;
//  String completeFileName = String(file_ptr->name());

 String completeFileName_String = String(file_ptr->name()); //filename cannot be too long

  int len = completeFileName_String.length();
  char completeFileName[len];
  completeFileName_String.toCharArray(completeFileName, len);

  if(!SD.exists(completeFileName)){
     Serial.println("\t" + completeFileName_String + " Doesn't exist");
  }
  
  //open file in read mode.
  if (temp = SD.open(completeFileName, FILE_READ)) {
    uint32_t size = temp.size();
    #ifdef SER_DEBUG
      Serial.println("------------------------------------------------------");
      Serial.println("File:\t" + completeFileName_String + "\t\tSize: " + String(size) + " Bytes");
      Serial.println("------------------------------------------------------");
    #endif
  }
  else {  //die
    #ifdef SER_DEBUG
      Serial.println("\t Failed to open " + completeFileName_String);
    #endif
    return;
  }

  while (temp.available()) {
    #ifdef SER_DEBUG
      Serial.write(temp.read());
    #endif
  }
  #ifdef SER_DEBUG
    Serial.println("------------------------------------------------------");
  #endif
  temp.close(); 
}

void readFile(const String filename_String){

  int len = filename_String.length();
  char filename[len];
  filename_String.toCharArray(filename, len);
  
  File file =  SD.open(filename, FILE_READ);
  readFile(&file);
  //call file destructor
}

//to be updated with time from RTC
String getTimeStamp(){
  return getRTCDateTime();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//          MY CODE FOR BACKUP RTC, Callibrated values              //////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//back up file will store last mission time:
bool doesBackUpExist(){
  if(SD.exists(backupFileName)){
#ifdef SER_DEBUG
      Serial.println("Found Backup.");
#endif
    return true;
  }else{
    //Then create a new backup File
#ifdef SER_DEBUG
    Serial.println("No backup file found, creating one");
#endif
    backup = SD.open(backupFileName,FILE_WRITE);
    backup.close();
    
    return false;
  }
}


void saveBackUp(){
  backup = SD.open(backupFileName,FILE_WRITE);
  if(backup){
#ifdef SER_DEBUG
    Serial.println("Storing All useful data to backup");
#endif
    backup.print(String(startTime)+","+String(ground_altitude)+","+String(mpu6050.getGyroXoffset())+","+String(mpu6050.getGyroYoffset())+","+String(mpu6050.getGyroZoffset())+","+String(dataPacket.software_state));
    backup.close();
  }else{
#ifdef SER_DEBUG
    Serial.println("No backup file Found");
#endif
  }
}

void backupUpdatedSoftwareState(int state){
  backup = SD.open(backupFileName,FILE_WRITE);
  if(backup){
#ifdef SER_DEBUG
    Serial.println("Storing new Software state for backup");
#endif
    backup.print(","+String(state));
    backup.close();
  }else{
#ifdef SER_DEBUG
    Serial.println("No backup file Found");
#endif
  }
}

void callibrateUsingPrevDatafromSD(){
#ifdef SER_DEBUG
    Serial.println("Getting last mission time from Backup");
#endif
  
  String text = "";
  backup = SD.open(backupFileName);

  while(backup.available()){
    char c = backup.read();
    text += c;
//    Serial.print(c);
  }
#ifdef SER_DEBUG
    Serial.println(text);
#endif  
  
  backup.close();
  
  //First Field is startTime of Mission from RTC
  int posl = 0;
  int posr = text.indexOf(",");
  String data = text.substring(posl,posr);
  setMissionTime(data.toInt());

  //Second Field is ground altitude from bmp:
  posl = posr+1;
  posr = text.indexOf(",",posl);
  data = text.substring(posl,posr);
  setGroundAltitude(data.toFloat());

  //Third, Fourth, fifth Field is ground Gyro X, Y, Z offset from MPU:
  posl = posr+1;
  posr = text.indexOf(",",posl);
  data = text.substring(posl,posr);
  float gyroXOff = data.toFloat();

  posl = posr+1;
  posr = text.indexOf(",",posl);
  data = text.substring(posl,posr);
  float gyroYOff = data.toFloat();

  posl = posr+1;
  posr = text.indexOf(",",posl);
  data = text.substring(posl,posr);
  float gyroZOff = data.toFloat();
  
  mpu6050.setGyroOffsets(gyroXOff,gyroYOff,gyroYOff);

  //Last Field will be software state:
  int lastPos = text.lastIndexOf(",");
  data = text.substring(lastPos+1);
  dataPacket.software_state = data.toInt();
  
}

bool doesBackUpPacketExist(){
  if(SD.exists(backupPacketName)){
#ifdef SER_DEBUG
      Serial.println("Found Backup of Packet Count.");
#endif
    return true;
  }else{
    //Then create a new backup Packet File
#ifdef SER_DEBUG
     Serial.println("No backup file for PACKETS found, creating one");
#endif  
    backupPacketCount = SD.open(backupPacketName,FILE_WRITE);
    backupPacketCount.close();
    
    return false;
  }
}

int setPacketCountFromSD(){
#ifdef SER_DEBUG
     Serial.println("Getting packet count from PacketFile");
#endif 
  
  String text = "";
  
  backupPacketCount = SD.open(backupPacketName);
  if(backupPacketCount){
    while(backupPacketCount.available()){
      char c = backupPacketCount.read();
      text += c;
//      Serial.print(c);
    }
    backupPacketCount.close();
  }else{
#ifdef SER_DEBUG
     Serial.println("No packetsFile object file found");
#endif 
    
    return 0;
  }
//  Serial.println(text);
  
  
  int pos = text.lastIndexOf(",");
  String packetCounter = text.substring(pos+1);
  Serial.println("PACKET COUNT WAS : "+packetCounter);
  return packetCounter.toInt();
}

void savePacketCount(){
  backupPacketCount = SD.open(backupPacketName,FILE_WRITE);
  if(backupPacketCount){
#ifdef SER_DEBUG
     Serial.println("Storing All packet count data to backupPacketCount");
#endif
    backupPacketCount.print(","+String(dataPacket.packet_count));
    backupPacketCount.close();
  }else{
#ifdef SER_DEBUG
     Serial.println("No backupPacketCount file Found");
#endif    
    
  }
}
