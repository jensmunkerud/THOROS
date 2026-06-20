#pragma once
#include "MISC/Datatypes.h"
#include <Arduino.h>
#include <functional>
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

constexpr int16_t RFD_TIMEOUT_MS {320};
constexpr int16_t PING_INTERVAL {500};
constexpr int16_t SEND_TELEMETRY_INTERVAL {50};
constexpr uint8_t RFD_SERIAL {2};


class RFD900 {
public:
	RFD900(Telemetry& tel, Drone& droneState);
	void begin();
	void loop();
	void sendTelemetry();
	void ping();
	QueueHandle_t getCommandQueue() const;

private:
	bool handlePidLine();
	bool isLikelyTextFrame() const;
	bool handleFramedPidPayload();
	void clearPidBuffer();
	char pidLineBuffer[128];
	size_t pidLineLength;

	static void RFD900Task(void* parameter);
	Telemetry& telemetry;
	Drone& drone;
	byte buffer[192];
	byte numPackets;
	unsigned long lastCommand;
	TaskHandle_t rfdTaskHandle;
	QueueHandle_t commandQueue;
	SemaphoreHandle_t serialTxMutex;
	HardwareSerial SerialRFD;
	uint16_t pingProgress;
	uint16_t telemetryProgress;
};