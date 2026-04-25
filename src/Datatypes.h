#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"

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

struct FlightControls {
	float pitch;
	float roll;
	float yaw;
	float throttle;
};

struct MotorOutputs {
	int16_t m1;
	int16_t m2;
	int16_t m3;
	int16_t m4;
};

enum class FlightMode : uint8_t {
		DISARMED,
		ARMED,
		LANDED,
		HOVERING,
		MOVING,
};

struct Drone {
	mutable portMUX_TYPE stateLock = portMUX_INITIALIZER_UNLOCKED;
	FlightMode mode;
	Attitude attitude;
	FlightControls flightControls;
	MotorOutputs motorOutputs;
	float altitude; // [m] above/under starting point

	bool GPS_OK;
	bool MOTOR_OK;
	bool PRESSURE_OK;
	bool IMU_OK;
	bool RADIO_OK;
	bool GROUND_LINK_OK;

	Drone() :
		mode(FlightMode::DISARMED),
		attitude{},
		flightControls{},
		motorOutputs{},
		altitude{},
		GPS_OK{false},
		MOTOR_OK{false},
		PRESSURE_OK{false},
		IMU_OK{false},
		RADIO_OK{false},
		GROUND_LINK_OK{false} {}

	inline void lock() {
		portENTER_CRITICAL(&stateLock);
	}

	inline void unlock() {
		portEXIT_CRITICAL(&stateLock);
	}
};

struct DroneLockGuard {
	Drone& drone;

	explicit DroneLockGuard(Drone& d) : drone(d) {
		drone.lock();
	}

	~DroneLockGuard() {
		drone.unlock();
	}
};

struct __attribute__((packed)) Telemetry {
	int8_t BEGIN;
	int16_t altitude;
	int16_t speed;
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
		linearAccel{},
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
