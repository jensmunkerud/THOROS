#pragma once
#include "Arduino.h"
#include "Status.h"
#include "MovementController.h"

constexpr int LEDPIN {2};
constexpr unsigned long FAST_INTERVAL = 1000/1;
constexpr unsigned long SLOW_INTERVAL = 1000/10;

class LED {
	public:
	LED(Telemetry& tel, MovementController& mc);
	void loop();

	private:
	Telemetry& telemetry;
	bool state;
	MovementController& mc;

};

LED::LED(Telemetry& tel, MovementController& mc) : telemetry{tel}, mc{mc}, state{false} {
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