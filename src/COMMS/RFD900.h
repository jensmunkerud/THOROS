#pragma once
#include "Datatypes.h"
#include <Arduino.h>
#include <functional>
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

constexpr int16_t RFD_TIMEOUT_MS {2500};
constexpr int16_t PING_INTERVAL {2000};
constexpr int16_t SEND_STATUS_INTERVAL {1500};
constexpr int16_t SEND_CONTROL_STREAM_INTERVAL {10};
constexpr uint8_t RFD_SERIAL {2};


class RFD900 {
public:
	typedef void (*ApplyPidCallback)(const PID& pitch, const PID& roll, const PID& yaw, void* context);
	void setPidApplyCallback(ApplyPidCallback callback, void* context = nullptr);

	RFD900(Telemetry& tel, Drone& droneState);
	void begin();
	void loop();
	void sendStatus();
	void sendControlStream();
	void ping();
	QueueHandle_t getCommandQueue() const;

private:
	bool handlePidLine();
	bool isLikelyTextFrame() const;
	bool handleFramedPidPayload();
	void clearPidBuffer();
	char pidLineBuffer[128];
	size_t pidLineLength;
	ApplyPidCallback pidCallback;
	void* pidContext;
	
	static void RFD900Task(void* parameter);
	Telemetry& telemetry;
	Drone& droneState;
	byte buffer[192];
	byte numPackets;
	unsigned long lastCommand;
	TaskHandle_t rfdTaskHandle;
	QueueHandle_t commandQueue;
	SemaphoreHandle_t serialTxMutex;
	HardwareSerial SerialRFD;
	uint16_t pingProgress;
	uint16_t statusProgress;
	uint16_t controlStreamProgress;
};