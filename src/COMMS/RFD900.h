#pragma once
#include "Status.h"
#include <Arduino.h>
#include <functional>
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"

constexpr int16_t RFD_TIMEOUT_MS {5000};
constexpr int16_t PING_INTERVAL {2000};
constexpr int16_t SEND_STATUS_INTERVAL {5};
constexpr uint8_t RFD_SERIAL {2};

void RFD900Task(void* parameter);

class RFD900 {
	public:
	RFD900(Status& s);
	void begin();
	void loop();
	void sendStatus();
	void ping();
	QueueHandle_t getCommandQueue() const;
	uint16_t pingProgress;
	uint16_t statusProgress;

	private:
	Status& status;
	byte buffer[32];
	byte numPackets;
	byte command_id;
	uint16_t value;
	unsigned long lastCommand;
	TaskHandle_t rfdTaskHandle;
	QueueHandle_t commandQueue;
	HardwareSerial SerialRFD;
};