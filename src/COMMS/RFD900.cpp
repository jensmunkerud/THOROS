#include "RFD900.h"
#include "MISC/MovementController.h"
#include "MISC/Motor.h"

#include <cstdio>

extern Motor motor;


RFD900::RFD900(Telemetry& tel, Drone& drone) :
	telemetry{tel},
	drone{drone},
	numPackets{0},
	pidLineLength{0},
	rfdTaskHandle{nullptr},
	serialTxMutex{nullptr},
	pingProgress{0},
	telemetryProgress{0},
	lastCommand{millis()},
	SerialRFD{RFD_SERIAL}
{
	commandQueue = xQueueCreate(5, sizeof(RFDCommandPacket));
	serialTxMutex = xSemaphoreCreateMutex();
	pidLineBuffer[0] = '\0';
}


// Task function (runs on Core 0)
void RFD900::RFD900Task(void* parameter) {
	RFD900* rfd = static_cast<RFD900*>(parameter);
	if (!rfd) {
		vTaskDelete(NULL); // safety check
	}
	{
		DroneLockGuard droneLock(rfd->drone);
		rfd->drone.RADIO_OK = true;
	}
	for (;;) {
		rfd->loop();
		vTaskDelay(1); // 1ms
		if (rfd->pingProgress++ >= PING_INTERVAL) {
			rfd->ping();
			rfd->pingProgress = 0;
		}
		if (rfd->telemetryProgress++ >= SEND_TELEMETRY_INTERVAL) {
			rfd->sendTelemetry();
			rfd->telemetryProgress = 0;
		}
	}
}

void RFD900::begin() {
	SerialRFD.begin(115200, SERIAL_8N1, 16, 17);
	if (serialTxMutex && xSemaphoreTake(serialTxMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
		SerialRFD.write(START_MARKER);
		SerialRFD.write(END_MARKER);
		xSemaphoreGive(serialTxMutex);
	}

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
	while (SerialRFD.available() > 0) {
		byte incoming = SerialRFD.read();

		// Shared marker-based frame parser (binary command packets and PID text frames)
		if (numPackets == 0) {
			if (incoming == START_MARKER) {
				buffer[numPackets++] = incoming;
				clearPidBuffer();
			}
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
			{
				DroneLockGuard droneLock(drone);
				drone.GROUND_LINK_OK = true;
			}
			lastCommand = millis();

			if (isLikelyTextFrame()) {
				handleFramedPidPayload();
				numPackets = 0;
				continue;
			}

			RFDCommandPacket packet;
			packet.numCmds = 0;
			// Parse pairwise
			for (int i = 1; i + 1 < numPackets - 1; i += 2) {
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

	// Failsafe is evaluated AFTER draining the RX buffer so that frames already
	// received (but not yet parsed, e.g. after a scheduling hiccup) count as link.
	if (millis() - lastCommand > RFD_TIMEOUT_MS) {
		bool hadGroundLink = false;
		{
			DroneLockGuard droneLock(drone);
			hadGroundLink = drone.GROUND_LINK_OK;
			drone.GROUND_LINK_OK = false;
		}
		if (hadGroundLink) {
			// KILLS DRONE ON RADIO TIMEOUT
			RFDCommandPacket killCommand{1, { {static_cast<uint8_t>(CommandID::KILL), 0} }};
			xQueueSendToBack(commandQueue, &killCommand, pdMS_TO_TICKS(10));
		}
	}
}

bool RFD900::isLikelyTextFrame() const {
	if (numPackets < 3) {
		return false;
	}

	// Tagged tuning frames ("O/", "I/", "F/", "R/") are text regardless of length.
	// The tag bytes do not collide with any binary CommandID value.
	char first = static_cast<char>(buffer[1]);
	if ((first == 'O' || first == 'I' || first == 'F' || first == 'R') && static_cast<char>(buffer[2]) == '/') {
		return true;
	}

	size_t slashCount = 0;
	for (size_t i = 1; i < static_cast<size_t>(numPackets - 1); ++i) {
		char ch = static_cast<char>(buffer[i]);
		if (ch == '/') {
			slashCount++;
		}
		if (ch == '\r' || ch == '\n') {
			continue;
		}
		if (ch < 32 || ch > 126) {
			return false;
		}
	}

	// PID and control stream payloads are slash-delimited text with many separators.
	return slashCount >= 6;
}


void RFD900::sendTelemetry() {
	bool linkOk = false;
	TelemetryPacket packet;
	{
		DroneLockGuard droneLock(drone);
		TelemetryLockGuard telemetryLock(telemetry);
		linkOk = drone.GROUND_LINK_OK;
		if (linkOk) {
			packet.drone = static_cast<const DroneData&>(drone);
			packet.telemetry = static_cast<const TelemetryData&>(telemetry);
		}
	}
	if (linkOk) {
		if (serialTxMutex && xSemaphoreTake(serialTxMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
			SerialRFD.write(reinterpret_cast<const uint8_t*>(&packet), TelemetryPacket::WIRE_SIZE);
			xSemaphoreGive(serialTxMutex);
		}
	}
}


void RFD900::ping() {
	if (serialTxMutex && xSemaphoreTake(serialTxMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
		SerialRFD.write(START_MARKER);
		SerialRFD.write(END_MARKER);
		xSemaphoreGive(serialTxMutex);
	}
}

QueueHandle_t RFD900::getCommandQueue() const { return commandQueue; }

bool RFD900::handlePidLine() {
	if (pidLineLength == 0) {
		clearPidBuffer();
		return false;
	}

	// "F/<value>" sets the front motor bias, "R/<value>" the right motor bias.
	if (pidLineLength >= 2 && pidLineBuffer[1] == '/' &&
		(pidLineBuffer[0] == 'F' || pidLineBuffer[0] == 'R')) {
		char tag = pidLineBuffer[0];
		float bias;
		int matched = sscanf(pidLineBuffer + 2, "%f", &bias);
		clearPidBuffer();
		if (matched != 1) {
			return false;
		}
		if (tag == 'F') {
			motor.setFrontBias(bias);
		} else {
			motor.setRightBias(bias);
		}
		return true;
	}

	// Optional loop tag prefix: "O/" targets the outer angle loop, "I/" the
	// inner rate loop. Untagged frames keep the legacy meaning (inner rate).
	const char* payload = pidLineBuffer;
	bool isOuterLoop = false;
	if (pidLineLength >= 2 && pidLineBuffer[1] == '/' &&
		(pidLineBuffer[0] == 'O' || pidLineBuffer[0] == 'I')) {
		isOuterLoop = (pidLineBuffer[0] == 'O');
		payload += 2;
	}

	float values[9];
	int matched = sscanf(
		payload,
		"%f/%f/%f/%f/%f/%f/%f/%f/%f",
		&values[0],
		&values[1],
		&values[2],
		&values[3],
		&values[4],
		&values[5],
		&values[6],
		&values[7],
		&values[8]
	);

	clearPidBuffer();

	if (matched != 9) {
		return false;
	}

	PID pitch{values[0], values[1], values[2]};
	PID roll{values[3], values[4], values[5]};
	PID yaw{values[6], values[7], values[8]};
	if (isOuterLoop) {
		motor.setAnglePidTunings(pitch, roll, yaw);
	} else {
		motor.setRatePidTunings(pitch, roll, yaw);
	}
	return true;
}

bool RFD900::handleFramedPidPayload() {
	if (numPackets < 3) {
		return false;
	}

	size_t payloadLength = static_cast<size_t>(numPackets - 2);
	if (payloadLength == 0 || payloadLength >= sizeof(pidLineBuffer)) {
		return false;
	}

	size_t slashCount = 0;
	size_t writeIndex = 0;
	for (size_t i = 1; i < static_cast<size_t>(numPackets - 1); ++i) {
		char ch = static_cast<char>(buffer[i]);
		if (ch == '/') {
			slashCount++;
		}

		if (ch == '\r' || ch == '\n') {
			continue;
		}

		bool isNumericChar = (ch >= '0' && ch <= '9') || ch == '.' || ch == '-' || ch == '+' || ch == '/' || ch == ' ';
		bool isTagChar = (writeIndex == 0) && (ch == 'O' || ch == 'I' || ch == 'F' || ch == 'R');
		if (!isNumericChar && !isTagChar) {
			return false;
		}

		if (writeIndex >= sizeof(pidLineBuffer) - 1) {
			clearPidBuffer();
			return false;
		}
		pidLineBuffer[writeIndex++] = ch;
	}

	// Bias frames carry a single value; PID frames need 9.
	size_t requiredSlashes = (pidLineBuffer[0] == 'F' || pidLineBuffer[0] == 'R') ? 1 : 8;
	if (slashCount < requiredSlashes) {
		clearPidBuffer();
		return false;
	}

	pidLineLength = writeIndex;
	pidLineBuffer[pidLineLength] = '\0';
	return handlePidLine();
}

void RFD900::clearPidBuffer() {
	pidLineLength = 0;
	pidLineBuffer[0] = '\0';
}

