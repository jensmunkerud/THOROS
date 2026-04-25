#pragma once
#include "Arduino.h"
#include "Datatypes.h"

constexpr int LEDPIN {2};
constexpr unsigned long BLINK_INTERVAL = 1000/4;

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
	bool droneArmed = false;
	{
		DroneLockGuard lock(drone);
		droneArmed = drone.mode != FlightMode::DISARMED;
	}
	if (droneArmed) {
		digitalWrite(LEDPIN, HIGH);
	} else {
		// BLINK IF ELSE
		digitalWrite(LEDPIN, state ? HIGH : LOW);
		state = !state;
	}
}