#pragma once

#include <Arduino.h>
#include "Status.h"

// Reusable callback for applying parsed PID values to a Motor instance passed via context.
void applyPidTuningsToMotor(const PID& pitch, const PID& roll, const PID& yaw, void* context);

class PidTuningReceiver {
public:
	typedef void (*ApplyCallback)(const PID& pitch, const PID& roll, const PID& yaw, void* context);

	PidTuningReceiver(Stream& serial, ApplyCallback callback, void* context = nullptr);
	void loop();

private:
	bool handleLine();

	Stream& serial;
	ApplyCallback callback;
	void* context;
	char lineBuffer[128];
	size_t lineLength;
};