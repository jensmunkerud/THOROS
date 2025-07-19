#include "NRF24.h"
#include <SPI.h>

NRF24::NRF24(Status& status) : radio{CE_PIN, CS_PIN}, status{status} {}


void NRF24::begin() {
	radio.begin();
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
	radio.setChannel(124);
	radio.setRetries(0, 1);

	if (IS_NODE_A) {
		radio.openWritingPipe(addressB);  // Node A sends to B
		radio.openReadingPipe(1, addressA); // Node A receives on A
	} else {
		radio.openWritingPipe(addressA);  // Node B sends to A
		radio.openReadingPipe(1, addressB); // Node B receives on B
	}

	radio.startListening();
	Serial.println(IS_NODE_A ? "Node A Ready" : "Node B Ready");
	radio.printDetails();
}

// Continously fetches any radio messages and calls callbackfunction if so
void NRF24::loop() {
	if (radio.available()) {
	// 	char buffer[32];
	// 	radio.read(&buffer, sizeof(buffer));
	// 	if (receiveCallback != nullptr) {
	// 		receiveCallback(buffer);
	// 	}
		radio.read(&status, sizeof(status));
	}
}


bool NRF24::send(const char data[]) {
	radio.stopListening();
	delayMicroseconds(100);
	bool success = radio.write(data, strlen(data) + 1);
	delayMicroseconds(100);
	// Serial.println(success ? "Send OK" : "Send failed");
	// if (success) {Serial.print("Sent: ");}
	return success;
}


void NRF24::setCallback(void (*callback)(const char* message)) {
	receiveCallback = callback;
}


bool NRF24::sendStatus() {
	radio.stopListening();
	delayMicroseconds(100);
	bool success = radio.write(&status, sizeof(status));
	delayMicroseconds(100);
	return success;
}