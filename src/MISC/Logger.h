#pragma once

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>

#include "Datatypes.h"

static constexpr int LOGGER_CS		{5};
static constexpr int LOGGER_SCK		{14};
static constexpr int LOGGER_MISO	{27};
static constexpr int LOGGER_MOSI	{13};

class Motor;

class Logger {
public:
	Logger(Drone& droneState, Telemetry& telemetryState);
	bool begin();
	bool startLog(bool includeFullTelemetry = false);
	void stopLog();
	bool isReady() const;
	bool isLogging() const;

private:
	struct LogSnapshot {
		float timestampSeconds;
		float deltaSeconds;
		FlightControls flightControls;
		Attitude attitude;
		Attitude commandOutput;
		MotorThrusts motorThrusts;
		TelemetryData telemetry;
		FlightMode mode;
		float altitude;
		bool gpsOk;
		bool motorOk;
		bool pressureOk;
		bool imuOk;
		bool radioOk;
		bool groundLinkOk;
	};

	bool buildLogPath(char* output, size_t outputSize) const;
	bool openLogFile();
	static void LOGGERTaskEntry(void* param);
	void LOGGERTask();
	void writeHeader();
	void writeBasicHeader();
	void writeFullTelemetryHeader();
	void writeSnapshot(const LogSnapshot& snapshot);
	void writeBasicSnapshot(const LogSnapshot& snapshot);
	void writeFullTelemetrySnapshot(const LogSnapshot& snapshot);
	void writeSeparator();
	void writeBool(bool value);
	void writeInt32(int32_t value);
	void writeUInt8(uint8_t value);
	void writeFloat(float value, uint8_t digits = 3);
	LogSnapshot captureSnapshot() const;

	Drone& drone;
	Telemetry& telemetry;
	File logFile;
	bool ready;
	bool logging;
	bool includeFullTelemetry;
	unsigned long logStartMs;
	unsigned long lastWriteMs;
	char logPath[80];
	SPIClass hspi;
	TaskHandle_t loggerTaskHandle;
};