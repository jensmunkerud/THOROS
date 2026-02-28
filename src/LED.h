#pragma once
#include "Arduino.h"
#include "Status.h"
#include "MovementController.h"

constexpr int LEDPIN {2};

class LED {
	public:
	LED(Status& s, MovementController& mc);
	void loop();

	private:
	Status& status;
	bool state;
	MovementController& mc;
};

LED::LED(Status& s, MovementController& mc) : status{s}, mc{mc}, state{false} {
	pinMode(LEDPIN, OUTPUT);
}

void LED::loop() {
	if (status.RFD900 == 1) {
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