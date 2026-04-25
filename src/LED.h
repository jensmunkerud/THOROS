#pragma once
#include "Arduino.h"
#include "Datatypes.h"

constexpr int LEDPIN {2};
constexpr unsigned long BLINK_INTERVAL = 1000/100;

class LED {
public:
	LED(Telemetry& tel, Drone& drone);
	void loop();

private:
	Telemetry& telemetry;
	Drone& drone;
	bool state;
	long lastTime;
};

LED::LED(Telemetry& tel, Drone& drone) : 
telemetry{tel},
drone{drone},
state{false}
{
	pinMode(LEDPIN, OUTPUT);
	lastTime = millis();
}

void LED::loop() {
	if (millis() - lastTime <= BLINK_INTERVAL) {return;}
	lastTime = millis();
	bool linkOk = false;
	{
		DroneLockGuard lock(drone);
		linkOk = drone.GROUND_LINK_OK;
	}
	if (linkOk) {
		// CONSTANT LIGHT IF COMMS CONNECTED
		digitalWrite(LEDPIN, HIGH);
	} else {
		// BLINK IF ELSE
		digitalWrite(LEDPIN, state ? HIGH : LOW);
		state != state;
	}
}