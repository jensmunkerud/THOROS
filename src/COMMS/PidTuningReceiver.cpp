#include "PidTuningReceiver.h"

#include <cstdio>
#include "../Motor.h"

void applyPidTuningsToMotor(const PID& pitch, const PID& roll, const PID& yaw, void* context) {
	Motor* targetMotor = static_cast<Motor*>(context);
	if (targetMotor == nullptr) {
		return;
	}

	targetMotor->setAttitudePidTunings(pitch, roll, yaw);
}

PidTuningReceiver::PidTuningReceiver(Stream& serial, ApplyCallback callback, void* context) :
serial{serial},
callback{callback},
context{context},
lineLength{0}
{
	lineBuffer[0] = '\0';
}


void PidTuningReceiver::loop() {
	while (serial.available() > 0) {
		char incoming = static_cast<char>(serial.read());
		if (incoming == '\r') {
			continue;
		}
		if (incoming == '\n') {
			handleLine();
			continue;
		}

		if (lineLength < sizeof(lineBuffer) - 1) {
			lineBuffer[lineLength++] = incoming;
			lineBuffer[lineLength] = '\0';
		} else {
			lineLength = 0;
			lineBuffer[0] = '\0';
		}
	}
}


bool PidTuningReceiver::handleLine() {
	if (lineLength == 0) {
		return false;
	}

	float values[9];
	int matched = sscanf(
		lineBuffer,
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
	lineLength = 0;
	lineBuffer[0] = '\0';

	if (matched != 9 || callback == nullptr) {
		return false;
	}

	PID pitch{values[0], values[1], values[2]};
	PID roll{values[3], values[4], values[5]};
	PID yaw{values[6], values[7], values[8]};
	callback(pitch, roll, yaw, context);
	return true;
}