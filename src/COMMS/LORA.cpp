#include <COMMS/LORA.h>


LORA::LORA() {
	// Binds commands to functions
	// commandMap["getpos"] = [this]()		{ getGPSPosition(); };
	commandMap["69"] = [this]()			{ Handshake(); };
	// commandMap["status"] = [this]()		{ sendStatus(); };
	commandMap["Power on"] = [this]()	{ setAntennaOK(); };
	pinMode(13, OUTPUT);
}


void LORA::loop() {
	if (Serial1.available()) {
		String received = Serial1.readStringUntil('\n');
		received.trim();
		parseMessage(received);
	}
}


void LORA::parseMessage(const String s) {
	auto it = commandMap.find(s);
	if (it != commandMap.end()) {
		it->second(); // Call the associated function
	} else {
		unknownCommand(s); // AT commands are interpreted here
	}
}


void LORA::setAntennaOK() {
	antennaOK = true;
	digitalWrite(13, HIGH);
}


void LORA::Handshake() {
	Serial1.println("420");
}


void LORA::unknownCommand(const String& cmd) {
	if (cmd.startsWith("AT")) {
		performATCommand(cmd);
	}
}


void LORA::performATCommand(const String& cmd) {
	Serial1.println("performing AT command...");
	delay(500);
	Serial1.println("+++"); // enters AT mode
	delay(500);
	Serial1.println(cmd);
	delay(500);
	unsigned long startTime = millis();
	String response = "";

	while (millis() - startTime < AT_Timeout) {
		while (Serial1.available()) {
			char c = Serial1.read();
			response += c;
			startTime = millis(); // Reset timeout on activity
		}
	}
	
	Serial1.println("+++"); // leaves AT mode
	delay(500);
	Serial1.println(response);
}