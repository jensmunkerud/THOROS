#include "transceiver.h"
#include <SPI.h>

Transceiver::Transceiver() : radio{CE_PIN, CSN_PIN} {
	Serial.begin(9600);
	radio.begin();
	radio.setPALevel(RF24_PA_MAX); // Adjust as needed
	radio.setDataRate(RF24_250KBPS);
	radio.setChannel(124);
	radio.setAutoAck(true);
	radio.setRetries(5, 15);  // More reliable

	if (IS_NODE_A) {
		radio.openWritingPipe(addressB);  // Node A sends to B
		radio.openReadingPipe(1, addressA); // Node A receives on A
	} else {
		radio.openWritingPipe(addressA);  // Node B sends to A
		radio.openReadingPipe(1, addressB); // Node B receives on B
	}

	radio.startListening();
	Serial.println(IS_NODE_A ? "Node A Ready" : "Node B Ready");
}

// Continously fetches any radio messages and calls callbackfunction if so
void Transceiver::loop() {
	if (radio.available()) {
		radio.read(&buffer, sizeof(buffer));
		if (receiveCallback != nullptr) {
			receiveCallback(buffer);
		}
	}
}


bool Transceiver::send(const char data[]) {
	radio.stopListening();
	bool success = radio.write(&data, strlen(data) + 1);
	radio.startListening();
	Serial.print("Sent: ");
	Serial.println(data);
	return success;
}

void Transceiver::setCallback(void (*callback)(const char* message)) {
	receiveCallback = callback;
}