#pragma once
#include <cstddef>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"

#define START_MARKER 0xAB
#define END_MARKER 0xCD

#define START_MARKER2 0xAD
#define END_MARKER2 0xFE

struct Vec3 {
	float x, y, z;
};

struct PID {
	float P, I, D;
};

struct __attribute__((packed)) Attitude {
	float pitch;
	float yaw;
	float roll;
};

struct __attribute__((packed)) FlightControls {
	float pitch;
	float roll;
	float yaw;
	float throttle;
};

struct __attribute__((packed)) MotorThrusts {
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

struct __attribute__((packed)) DroneData {
	FlightMode mode;
	Attitude attitude;				// deg
	Attitude gyroRate;				// deg/s
	MotorThrusts motorThrusts;
	FlightControls flightControls;
	float altitude;					// [m] above/under starting point

	bool GPS_OK;
	bool MOTOR_OK;
	bool PRESSURE_OK;
	bool IMU_OK;
	bool RADIO_OK;
	bool GROUND_LINK_OK;
	bool LOGGER_OK;
};

struct Drone : public DroneData {
	mutable portMUX_TYPE stateLock = portMUX_INITIALIZER_UNLOCKED;

	Drone() : DroneData{} {
		mode = FlightMode::DISARMED;
	}

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

struct __attribute__((packed)) TelemetryData {
	int16_t temp;
	int16_t pressure;
	int16_t batteryVoltage;
	bool loggerIsLogging;

	int32_t latitude;   // scaled by 1e7
	int32_t longitude;  // scaled by 1e7
};

struct Telemetry : public TelemetryData {
	mutable portMUX_TYPE stateLock = portMUX_INITIALIZER_UNLOCKED;

	Telemetry() : TelemetryData{} {}

	inline void lock() {
		portENTER_CRITICAL(&stateLock);
	}

	inline void unlock() {
		portEXIT_CRITICAL(&stateLock);
	}
};

struct TelemetryLockGuard {
	Telemetry& telemetry;

	explicit TelemetryLockGuard(Telemetry& t) : telemetry(t) {
		telemetry.lock();
	}

	~TelemetryLockGuard() {
		telemetry.unlock();
	}
};

struct TelemetryPacket {
	int8_t BEGIN;
	DroneData drone;
	TelemetryData telemetry;
	int8_t END;

	static constexpr size_t WIRE_SIZE =
		sizeof(BEGIN) +
		sizeof(drone) +
		sizeof(telemetry) +
		sizeof(END);

	TelemetryPacket() :
		BEGIN(START_MARKER2),
		drone{},
		telemetry{},
		END(END_MARKER2) {}
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
