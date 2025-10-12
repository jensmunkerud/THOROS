#pragma once
#include "Status.h"
#include "../MovementController.h"
#include <Arduino.h>
// #include <map>
#include <functional>

constexpr int8_t RFD_TIMEOUT_MS = 10000;
constexpr int8_t RFD_HANDSHAKETIME = 10000;

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
	int16_t value;
	int8_t timer;
	// std::map<byte, std::function<void(int16_t value)>> commandMap;
	MovementController movementController;
};