//Variable to store startTime of mission
uint32_t startTime = 1546;  //Monday, January 7, 2019 12:09:02 AM GMT+05:30

//Called in Setup to initialize RTC
void initRTC(){
   Serial.println("Init RTC");
  Serial.println(rtc.begin());
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}

void resetMissionTime(){
  DateTime now = rtc.now();
  startTime = now.unixtime();
}
//Returns the mission time: i.e. subtraction of "current unix time" and program's "startTime"
uint32_t getMissionTime(){
  DateTime now = rtc.now();
  return (now.unixtime() - startTime);
}

inline String getRTCDate(){
  DateTime now = rtc.now();
  return (String(now.day()) + "/" + String(now.month()) + "/" + String(now.year()));
}

inline String getRTCTime(){
  DateTime now = rtc.now();
  return (String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()));
}

String getRTCDateTime(){  //UTC
  DateTime now = rtc.now();
  String YYYY = String("20" + twoDigits(now.year()));
  String MM   = twoDigits(now.month());
  String DD   = twoDigits(now.day());
  String HH   = twoDigits(now.hour());
  String mm   = twoDigits(now.minute());
  String SS   = twoDigits(now.second());
  String TZ   = "+5:30";  //IST
  
  return String(YYYY+"-"+MM+"-"+DD+"T"+HH+":"+mm+":"+SS+TZ);
}

String twoDigits(int x){
  String s = "";
  if(x<10)
    s = "0";
  return String(s + String(x));
}

