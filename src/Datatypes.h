#pragma once
#include <Arduino.h>

#define START_MARKER 0xAB
#define END_MARKER 0xCD

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

struct ControlInput {
	float pitch		{0};
	float roll		{0};
	float yaw 		{0};
	float throttle	{0};
};

struct DroneState {
	FlightMode mode;
	ControlInput controlInput;
	int16_t altitude;

	bool gpsFix;
	bool motorArmed;
	bool BMP390;
	bool ICM20948;
	bool RFD900;
	bool Communication;

	DroneState() :
		mode(FlightMode::DISARMED),
		controlInput{} {}
};

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
