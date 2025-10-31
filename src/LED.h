#pragma once
#include "Arduino.h"
#include "Status.h"

static constexpr int LEDPIN {2};

class LED {
	public:
	LED(Status& s);
	void loop();

	private:
	Status& status;
	bool state;
};

LED::LED(Status& s) : status{s}, state{false} {pinMode(LEDPIN, OUTPUT);}

void LED::loop() {
	if (status.RFD900 == 1) {
		digitalWrite(LEDPIN, HIGH);
	} else {
		digitalWrite(LEDPIN, state ? HIGH : LOW);
		state = not state;
	}
}