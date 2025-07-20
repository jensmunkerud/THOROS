#pragma once
#include <RF24.h>
#include "Status.h"

constexpr int CE_PIN {10};
constexpr int CS_PIN {9};
const byte addressA[5] = {'N', 'A', '0', '0', '1'};
const byte addressB[5] = {'N', 'B', '0', '0', '2'};
constexpr bool IS_NODE_A{true};  // Change to false for the other Arduino


class NRF24 {
	public:
	NRF24(Status& status);
	void begin();
	void loop();
	bool send(const char data[]);
	void setCallback(void (*callback)(const int command, const int value));
	void receive();
	bool sendStatus();
	RF24 radio;

	private:
	void (*receiveCallback)(const int command, const int value) = nullptr;
	Status& status;
};