String packetFileName = "packets";
String packetFileExt  = "csv";
String completePacketFileName;

String missionFileName = "mission";
String missionFileExt = "log";
String completeMissionFileName;

String timeStamp = "2013-10-22T01:37:56+05:30";  //UTC Format: 1994-11-05T08:15:30-05:00 corresponds to November 5, 1994, 8:15:30 am, US Eastern Standard Time.

 /*
  Sets up the SD card and creates mission.log and packets.csv
  */
void initSD(){ 
  //----------Setting up sd card-----------//
  
  Serial.print("Initializing SD card...");
  
  
  if (!SD.begin(SD_SELECT)) {
    Serial.println("initialization failed!");
    // don't do anything more:
//    while (1);
  }

  Serial.println("initialization done.");

  //initializing packetFile and missionLog objects declared in TEENSY_prototype
  packetFile  = newFile(packetFileName, packetFileExt);
  missionLog  = newFile(missionFileName, missionFileExt); 
  
}

 /*
  Creates and sets up a new file.
  */
  
File newFile(String fileName, String fileExt){
  File tempFile;
 //completeFileName = String(fileName + "_" + String(millis()) + "." + fileExt);
  String completeFileName_String = String(fileName + "." + fileExt); //filename cannot be too long

  int len = completeFileName_String.length();
  char completeFileName[len];
  completeFileName_String.toCharArray(completeFileName, len);
  
//  delete any pre-existing record
  if(SD.exists(completeFileName)){
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
  else {  //die
    #ifdef SER_DEBUG
      Serial.println("Failed to create " + completeFileName_String);
    #endif
    return *(new File());  //return an empty file object
  }
  tempFile.close(); 
  return (tempFile);
}
boolean logEvent(const String text){
  /*
  Safely stores the text to mission.log with time stamp
  */
  return safe_println(&missionLog, String(getTimeStamp() + "\t" + text));
}

boolean safe_print(File* file_ptr, const String text){
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

boolean safe_println(File* file_ptr, const String text){
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
