#include "NRF24.h"
#include <SPI.h>

NRF24::NRF24() : radio{CE_PIN, CSN_PIN} {}

void NRF24::begin() {
	radio.begin();
	radio.setPALevel(RF24_PA_MAX); // Adjust as needed
	radio.setDataRate(RF24_250KBPS);
	radio.setChannel(124);
	radio.setAutoAck(true);
	radio.setRetries(0, 15);  // More reliable
	radio.enableAckPayload();
	radio.enableDynamicPayloads();
	radio.setAutoAck(true);

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
	radio.startListening();
	if (radio.available()) {
		char buffer[32];
		radio.read(&buffer, sizeof(buffer));
		if (receiveCallback != nullptr) {
			receiveCallback(buffer);
		}
	}
}


bool NRF24::send(const char data[]) {
	radio.stopListening();
	delayMicroseconds(100);
	bool success = radio.write(data, strlen(data) + 1);
	radio.startListening();
	delayMicroseconds(100);
	Serial.println(success ? "Send OK" : "Send failed");
	if (success) {Serial.print("Sent: ");}
	return success;
}

void NRF24::setCallback(void (*callback)(const char* message)) {
	receiveCallback = callback;
}