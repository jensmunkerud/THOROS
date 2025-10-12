#include "RFD900.h"


RFD900::RFD900(Status& s, MovementController& m) : status{s}, movementController{m} {
	// commandMap[100] = [this](int16_t value)			{ FORWARD(value); }; // FORWARD
	// commandMap[102] = [this](int16_t value)			{ none(value); }; // LEFT
	// commandMap[101] = [this](int16_t value)			{ none(value); }; // BACKWARD
	// commandMap[103] = [this](int16_t value)			{ none(value); }; // RIGHT
	// commandMap[104] = [this](int16_t value)			{ none(value); }; // PAN LEFT
	// commandMap[105] = [this](int16_t value)			{ none(value); }; // PAN RIGHT
	// commandMap[107] = [this](int16_t value)			{ none(value); }; // GO UP
	// commandMap[108] = [this](int16_t value)			{ none(value); }; // GO DOWN
}

void RFD900::begin() {
	Serial1.begin(57600);
	Serial1.write(START_MARKER);
	Serial1.write(HANDSHAKE);
	Serial1.write(END_MARKER);
	timer = 0;
	while (status.RFD900 != 1 && timer < RFD_TIMEOUT_MS) {
		loop();
		delay(1);
		timer++;
	}
	if (timer >= RFD_TIMEOUT_MS) {
		// SHIT AIN HANDSHAKIN
	}
}

void RFD900::loop() {
	index = 0;

	while (Serial1.available() > 0) {
		byte incoming = Serial1.read();
		// LOOKS FOR START_MARKER
		if (index == 0 && incoming != START_MARKER) {
			continue;
		}

		buffer[index++] = incoming;

		// WHEN BUFFER IS FULL
		if (index == 5) {
			if (buffer[0] == START_MARKER && buffer[4] == END_MARKER) {
				// Extract fields
				command_id = buffer[1];
				value = buffer[2] | (buffer[3] << 8); // little endian
				movementController.executeCommand(static_cast<CommandID>(command_id), value);
			}
		}

		// USED FOR CONFIRMING HANDSHAKE FROM BEGIN
		if (index == 1) {
			if (buffer[index] == HANDSHAKE) {
				status.RFD900 = 1;
			}
		}

		if (index > 5) {
			index = 0;
		}
	}
}


void RFD900::sendStatus() {
	if (status.RFD900 == 1) {
		Serial1.write((uint8_t*)&status, sizeof(Status));
	}
}

void none(int16_t value) {
	//
}

void FORWARD(int16_t value) {

}