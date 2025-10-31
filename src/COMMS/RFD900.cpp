#include "RFD900.h"


RFD900::RFD900(Status& s, MovementController& m) : status{s}, movementController{m} {}

void RFD900::begin() {
	Serial1.begin(57600);
	Serial1.write(START_MARKER);
	Serial1.write(HANDSHAKE);
	Serial1.write(END_MARKER);
	while (status.RFD900 != 1) {
		loop();
	}
}

void RFD900::loop() {
	index = 0;

	if (millis() - lastCommand > RFD_TIMEOUT_MS) {status.RFD900 = 0;}

	while (Serial1.available() > 0) {
		byte incoming = Serial1.read();

		// Wait for start marker
		if (index == 0 && incoming != START_MARKER) {
			continue;
		}

		buffer[index++] = incoming;

		// Safety: if index exceeds buffer size, reset
		if (index >= sizeof(buffer)) {
			index = 0;
			continue;
		}

		// Once a full 4-byte packet is received
		if (index == 4 && buffer[0] == START_MARKER && buffer[3] == END_MARKER) {
			lastCommand = millis();
			byte cmd = buffer[1];
			byte val = buffer[2];

			if (cmd == HANDSHAKE) {
				status.RFD900 = 1;
				movementController.begin();
			} else {
				// Protect against invalid or too-frequent commands
				movementController.executeCommand(static_cast<CommandID>(cmd), val);
			}

			index = 0; // reset for next packet
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
