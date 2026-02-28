#pragma once
#include "Status.h"
#include <Arduino.h>
#include <functional>
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"

constexpr int16_t RFD_TIMEOUT_MS {2100};
constexpr int16_t PING_INTERVAL {1000};
constexpr int16_t SEND_STATUS_INTERVAL {100};
constexpr uint8_t RFD_SERIAL {2};


class RFD900 {
	public:
	RFD900(Status& s);
	void begin();
	void loop();
	void sendStatus();
	void ping();
	QueueHandle_t getCommandQueue() const;

	private:
	static void RFD900Task(void* parameter);
	Status& status;
	byte buffer[32];
	byte numPackets;
	unsigned long lastCommand;
	TaskHandle_t rfdTaskHandle;
	QueueHandle_t commandQueue;
	HardwareSerial SerialRFD;
	uint16_t pingProgress;
	uint16_t statusProgress;
};