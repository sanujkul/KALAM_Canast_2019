//Variable to store startTime of mission
uint32_t startTime = 1546800011;	//Monday, January 7, 2019 12:09:02 AM GMT+05:30
/*
 * Function : To initialize the rtc object that was created in cansat_prototype_1.ino
 *             It set the epoch time to 0;
 */
/*
getEpoch() function returns the UNIX epoch time, ie the number of seconds since Jan 1 1970, provided it has been set accurately first.
So we will be using that as a permanent millis() function.
*/

void resetMissionTime(){
	startTime = rtc.getEpoch();
}
//Returns the mission time: i.e. subtraction of "current unix time" and program's "startTime"
uint32_t getMissionTime(){
  return (rtc.getEpoch() - startTime);
}

inline String getRTCDate(){
  return (String(rtc.getDay()) + "/" + String(rtc.getMonth()) + "/" + String(rtc.getYear()));
}

inline String getRTCTime(){
  return (String(rtc.getHours()) + ":" + String(rtc.getMinutes()) + ":" + String(rtc.getSeconds()));
}

String getRTCDateTime(){	//UTC
	String YYYY = String("20" + twoDigits(rtc.getYear()));
	String MM 	= twoDigits(rtc.getMonth());
	String DD 	= twoDigits(rtc.getDay());
	String HH 	= twoDigits(rtc.getHours());
	String mm 	= twoDigits(rtc.getMinutes());
	String SS 	= twoDigits(rtc.getSeconds());
	String TZ 	= "+5:30";	//IST
	
	return String(YYYY+"-"+MM+"-"+DD+"T"+HH+":"+mm+":"+SS+TZ);
}

String twoDigits(int x){
	String s = "";
	if(x<10)
		s = "0";
	return String(s + String(x));
}