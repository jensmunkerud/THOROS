#pragma once
#include <Arduino.h>

#define START_MARKER 0xAB
#define END_MARKER 0xCD
#define HANDSHAKE 0x10

struct Vec3 {
	float x, y, z;
};

struct PID {
	float P, I, D;
};

struct Attitude {
	float pitch;
	float yaw;
	float roll;
};

enum class FlightMode {
		DISARMED,
		ARMED,
		LANDED,
		HOVER,
		MOVING,
};

// Rename existing telemetry -> telemetry, then make a new struct for ONLY INTERNAL properties, like attitude & linearaccel etc.

struct __attribute__((packed)) Telemetry {
	int8_t BEGIN;
	int16_t altitude;
	int16_t speed;

	Attitude attitude;
	Vec3 linearAccel;

	int16_t temp;
	int16_t pressure;

	int16_t batteryVoltage;

	int32_t latitude;   // scaled by 1e7
	int32_t longitude;  // scaled by 1e7

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
		uint8_t RFD900			: 1;
		uint8_t Communication	: 1;
	};
	};

	uint16_t timestamp; // unused
	int8_t END;

	Telemetry() :
		BEGIN(START_MARKER),
		altitude(0),
		speed(0),
		// accelX(0), accelY(0), accelZ(0),
		attitude{0.0, 0.0, 0.0},
		linearAccel{0.0, 0.0, 0.0},
		temp(0), pressure(0),
		batteryVoltage(0),
		latitude(0), longitude(0),
		flags(0),
		timestamp(0),
		END(END_MARKER) {}
};


struct RFDCommand {
	uint8_t command;
	uint8_t value;
};

constexpr uint8_t MAX_COMMANDS_PER_PACKET = 15;

struct RFDCommandPacket {
	uint8_t numCmds;
	RFDCommand commands[MAX_COMMANDS_PER_PACKET];
};
