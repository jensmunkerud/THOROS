#pragma once
#include "Status.h"
#include "../MovementController.h"
#include <Arduino.h>
// #include <map>
#include <functional>

constexpr int16_t RFD_TIMEOUT_MS {5000};

class RFD900 {
	public:
	RFD900(Status& s, MovementController& m);
	void begin();
	void loop();
	void sendStatus();

	private:
	Status& status;
	byte buffer[5];
	byte index;
	byte command_id;
	uint16_t value;
	unsigned long lastCommand;
	MovementController movementController;
};