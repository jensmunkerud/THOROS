#include "RFD900.h"


RFD900::RFD900(Status& s) : status{s}, numPackets{0}, rfdTaskHandle{nullptr}, pingProgress{0}, SerialRFD(RFD_SERIAL) {
	commandQueue = xQueueCreate(5, sizeof(RFDCommandPacket));
}


// Task function (runs on Core 0)
void RFD900::RFD900Task(void* parameter) {
	RFD900* rfd = static_cast<RFD900*>(parameter);
	if (!rfd) {
		vTaskDelete(NULL); // safety check
	}
	Serial.begin(115200);
	for (;;) {
		rfd->loop();
		vTaskDelay(1); // 1ms
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
	SerialRFD.begin(115200, SERIAL_8N1, 16, 17);
	SerialRFD.write(START_MARKER);
	SerialRFD.write(END_MARKER);

	xTaskCreatePinnedToCore(
		RFD900Task,         // Task function
		"RFD900 Task",      // Name
		8192,               // Stack size (bytes)
		this,               // Parameter
		1,                  // Priority
		&rfdTaskHandle,     // Task handle
		0                   // Core 0
	);
}

void RFD900::loop() {
	if (millis() - lastCommand > RFD_TIMEOUT_MS) {
		if (status.RFD900 == 1) {
			RFDCommandPacket killCommand{1, { {254, 0} }};
			xQueueSendToBack(commandQueue, &killCommand, pdMS_TO_TICKS(10));
		}
		status.RFD900 = 0;
	}

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

		// Once a complete packet is confirmed
		// SOMETIMES they are bundled together, hence END_MARKER and START_MARKER skips inside
		// (it might send too fast for SerialRFD to timeout or we receive next packet while inside for loop below)
		if (incoming == END_MARKER) {
			status.RFD900 = 1;
			lastCommand = millis();
			RFDCommandPacket packet;
			packet.numCmds = 0;
			// Parse pairwise
			for (int i = 1; i < numPackets - 1; i += 2) {
				if (packet.numCmds >= MAX_COMMANDS_PER_PACKET) {break;}
				if (buffer[i] == END_MARKER || buffer[i] == START_MARKER) {continue;}
				packet.commands[packet.numCmds].command = buffer[i];
				packet.commands[packet.numCmds].value   = buffer[i + 1];
				packet.numCmds++;
			}
			// Send entire packet to queue
			xQueueSendToBack(commandQueue, &packet, pdMS_TO_TICKS(10));
			numPackets = 0;
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
	SerialRFD.write(END_MARKER);
}

QueueHandle_t RFD900::getCommandQueue() const { return commandQueue; }

void none(int16_t value) {
	//
}
