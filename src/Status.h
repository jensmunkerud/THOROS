#pragma once
#include <Arduino.h>

struct __attribute__((packed)) Status {
	int8_t BEGIN;
	int16_t altitude;
	int16_t speed;

	int16_t accelX;
	int16_t accelY;
	int16_t accelZ;

	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
	
	int16_t temp;
	int16_t pressure;

	int16_t batteryVoltage;

	// System flags (packed into 1 byte)
	union {
	uint8_t flags;
	struct {
		uint8_t systemsOk		: 1;
		uint8_t criticalOk		: 1;
		uint8_t gpsFix			: 1;
		uint8_t motorArmed		: 1;
		uint8_t BMP390			: 1;
		uint8_t ICM20948		: 1;
		uint8_t reserved		: 2;
	};
	};

	uint16_t timestamp; // unused
	int8_t END;

	Status() :
		BEGIN(0xAB),
		altitude(0),
		speed(0),
		accelX(0), accelY(0), accelZ(0),
		gyroX(0), gyroY(0), gyroZ(0),
		temp(0), pressure(0),
		batteryVoltage(0),
		flags(0),
		timestamp(0),
		END(0xCD) {}
};


