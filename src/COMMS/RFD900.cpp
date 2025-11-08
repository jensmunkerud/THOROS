#include "RFD900.h"


RFD900::RFD900(Status& s) : status{s}, numPackets{0}, rfdTaskHandle{nullptr}, pingProgress{0}, SerialRFD(RFD_SERIAL) {
	commandQueue = xQueueCreate(10, 2 * sizeof(uint8_t));
}


// Task function (runs on Core 0)
void RFD900Task(void* parameter) {
	RFD900* rfd = static_cast<RFD900*>(parameter);
	if (!rfd) {
		vTaskDelete(NULL); // safety check
	}
	for (;;) {
		rfd->loop();
		vTaskDelay(1);
		if (rfd->pingProgress++ >= PING_INTERVAL) {
			rfd->ping();
			rfd->pingProgress = 0;
		}
		if (rfd->statusProgress++ >= SEND_STATUS_INTERVAL) {
			rfd->sendStatus();
			rfd->statusProgress = 0;
		}
	}
}

void RFD900::begin() {
	SerialRFD.begin(57600, SERIAL_8N1, 16, 17);
	SerialRFD.write(START_MARKER);
	SerialRFD.write(HANDSHAKE);
	SerialRFD.write(END_MARKER);

	// xTaskCreatePinnedToCore(
	// RFD900Task,         // Task function
	// "RFD900 Task",      // Name
	// 4096,               // Stack size (bytes)
	// this,               // Parameter
	// 1,                  // Priority
	// &rfdTaskHandle,     // Task handle
	// 0                   // Core 0
	// );

	// while (status.RFD900 != 1) {
	// 	loop();
	// 	delay(1);
	// }
}

void RFD900::loop() {

	if (millis() - lastCommand > RFD_TIMEOUT_MS) {status.RFD900 = 0;}

	while (SerialRFD.available() > 0) {
		byte incoming = SerialRFD.read();

		// Wait for start marker
		if (numPackets == 0 && incoming != START_MARKER) {
			continue;
		}
		buffer[numPackets++] = incoming;

		// Safety: if numPackets exceeds buffer size, reset
		if (numPackets >= sizeof(buffer)) {
			numPackets = 0;
			continue;
		}

		// Once a full 4-byte packet is received
		if (numPackets == 4 && buffer[0] == START_MARKER && buffer[3] == END_MARKER) {
			lastCommand = millis();
			byte cmd = buffer[1];
			byte val = buffer[2];

			if (cmd == HANDSHAKE) {
				status.RFD900 = 1;
			} else {
				// Protect against invalid or too-frequent commands
				// movementController.executeCommand(static_cast<CommandID>(cmd), val);
				uint8_t packet[2] = {cmd, val};
				xQueueSendToBack(commandQueue, packet, pdMS_TO_TICKS(10));  // non-blocking
			}

			numPackets = 0; // reset for next packet
		}
	}
}


void RFD900::sendStatus() {
	if (status.RFD900 == 1) {
		SerialRFD.write((uint8_t*)&status, sizeof(Status));
	}
}


void RFD900::ping() {
	SerialRFD.write(START_MARKER);
	SerialRFD.write(HANDSHAKE);
	SerialRFD.write(END_MARKER);
}

QueueHandle_t RFD900::getCommandQueue() const { return commandQueue; }

void none(int16_t value) {
	//
}
