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
	float pitch	{0};
	float yaw	{0};
	float roll	{0};
};

struct FlightControls {
	float pitch		{0};
	float roll		{0};
	float yaw 		{0};
	float throttle	{0};
};

enum class FlightMode {
		DISARMED,
		ARMED,
		LANDED,
		HOVERING,
		MOVING,
};

struct Drone {
	FlightMode mode;
	FlightControls flightControls;
	int16_t altitude;

	bool GPS_OK;
	bool MOTOR_ARMED;
	bool PRESSURE_OK;
	bool IMU_OK;
	bool RADIO_OK;
	bool GROUND_LINK_OK;

	Drone() :
		mode(FlightMode::DISARMED),
		flightControls{},
		altitude{false},
		GPS_OK{false},
		MOTOR_ARMED{false},
		PRESSURE_OK{false},
		IMU_OK{false},
		RADIO_OK{false},
		GROUND_LINK_OK{false} {}
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
		attitude{},
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
