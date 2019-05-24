
boolean savePacket(packet* dataPacket){
	/*
	Stores packet to SD Card
	*/
	return safe_println(&packetFile, dataPacket->toString());
}

packet* getLastPacket(File* packetFile){
	/*
	Reads SD card and creates a new packet from the last line.
	*/
	packet* newPacket = new packet(TEAM_ID);
	
	//Here's how I would have done it in a scripting language
	/*
		File csvFile = SD.open(packetFile->name(), FILE_WRITE);
		int cursor = csvFile.getCursor();
		csvFile.close();
		csvFile = SD.open(packetFile->name(), FILE_READ);
		csvFile.setCursor(cursor-1);
		String line = csvFile.readLine();
		String fields[] = line.split(',');
		newPacket->teamID = fields[0].toInt();
		.
		.
		newPacket->pressure = fields[4].toFloat();
		.
		.
		.
	*/
	return newPacket;
}

packet* getPacket(File* packetFile, int line){
	/*
	Reads SD card and creates a new packet from a specific line
	*/
	packet* newPacket = new packet(TEAM_ID);
	return newPacket;	
}

boolean transmitPacketString(packet* dataPacket){
	/*
	Transmits packet as a csv string over zigbee
	*/
	xbee.println(dataPacket->toString());
  return true;
}
