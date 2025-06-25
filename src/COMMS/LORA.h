#include <Arduino.h>
#include <map>
#include <functional>
class GPS {};

class LORA {
	public:
	LORA();
	void parseMessage(String s);
	void loop();

	private:
	std::map<String, std::function<void()>> commandMap;
	GPS gps;
	// COMMANDS
	void getGPSPosition();					// getpos -> returns GPS location ONCE
	void Handshake();				// 420    -> returns 420+69 = 489  used for handshaking
	void sendStatus();
	void setAntennaOK();
	void unknownCommand(const String& cmd);
	bool antennaOK = false;

	void performATCommand(const String& cmd);
	const unsigned long AT_Timeout {1000};
};