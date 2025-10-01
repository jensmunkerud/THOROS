#include "NRF24.h"
#include <SPI.h>

NRF24::NRF24(Status& status) : radio{CE_PIN, CS_PIN}, status{status} {}


void NRF24::begin() {
	radio.begin();
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
	radio.setChannel(124);
	radio.setRetries(0, 1);
	radio.setAutoAck(true);

	if (IS_NODE_A) {
		radio.openWritingPipe(addressB);  // Node A sends to B
		radio.openReadingPipe(1, addressA); // Node A receives on A
	} else {
		radio.openWritingPipe(addressA);  // Node B sends to A
		radio.openReadingPipe(1, addressB); // Node B receives on B
	}

	radio.startListening();
}

// Continously fetches for COMMANDS
void NRF24::loop() {
	if (radio.available()) {
		uint8_t packet[5];
		radio.read(&packet, sizeof(packet));

		if (packet[0] == 0xAB && packet[4] == 0xCD) { // Check start & end markers
			uint8_t command = packet[1];
			int16_t value = packet[2] | (packet[3] << 8); // Combine little-endian
			receiveCallback(command, value);
			Serial.println("I understood everything");
		} else {
			Serial.println("NO HABLO INGLES");
		}
	}
}


void NRF24::setCallback(void (*callback)(const int command, const int value)) {
	receiveCallback = callback;
}


bool NRF24::sendStatus() {
	radio.stopListening();
	delayMicroseconds(100);
	bool success = radio.write(&status, sizeof(status));
	delayMicroseconds(100);
	radio.startListening();
	return success;
}