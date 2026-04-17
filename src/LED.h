#pragma once
#include "Arduino.h"
#include "Datatypes.h"
#include "MovementController.h"

constexpr int LEDPIN {2};
constexpr unsigned long FAST_INTERVAL = 1000/1;
constexpr unsigned long SLOW_INTERVAL = 1000/10;

class LED {
public:
	LED(Telemetry& tel, DroneState& droneState, MovementController& mc);
	void loop();

private:
	Telemetry& telemetry;
	DroneState& droneState;
	bool state;
	MovementController& mc;
	long lastFast;
	long lastSlow;
};

LED::LED(Telemetry& tel, DroneState& droneState, MovementController& mc) : 
telemetry{tel},
droneState{droneState},
state{false},
mc{mc}
{
	pinMode(LEDPIN, OUTPUT);
}

void LED::loop() {
	
	if (telemetry.Communication == 1) {
		// CONSTANT LIGHT IF COMMS CONNECTED
		digitalWrite(LEDPIN, HIGH);
	} else {
		// BLINK IF ELSE
		digitalWrite(LEDPIN, state ? HIGH : LOW);
		state = not state;
	}

	// for (bool elem : mc.commands_in_action) {
	// 	state = false;
	// 	if (elem) {
	// 		state = true;
	// 	}
	// 	digitalWrite(LEDPIN, state ? HIGH : LOW);
	// }
}