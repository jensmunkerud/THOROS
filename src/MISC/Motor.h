#pragma once
#include <Arduino.h>
#include <DShotRMT.h>
#include <QuickPID.h>
#include "MovementController.h"
#include "Datatypes.h"
#include "Filters.h"

// CONFIGURATION
constexpr int MOTOR1 {32}; // FRONT RIGHT	CCW
constexpr int MOTOR2 {33}; // BACK RIGHT	CW
constexpr int MOTOR3 {25}; // FRONT LEFT	CW
constexpr int MOTOR4 {26}; // BACK LEFT		CCW

constexpr int INITILIZE_ESC_TIME {40}; // 4000
constexpr int MIN_ARMED_DSHOT_VALUE {0}; // 0-47 is dShot reserved values
constexpr int MAXIMUM_MOTOR_SPEED {500};
constexpr float MOTOR_KILL_SPEED {1500.0f};	// [units/s]
constexpr float MAX_DISARM_TILT_ANGLE_DEG {30.0f};
constexpr float FRONT_BIAS {1.1f};

constexpr int PID_MAX_EFFECT_AFTER_SPEED {200};
constexpr uint32_t OUTER_PID_INTERVAL_US {10000};		// 100 Hz
constexpr uint32_t INNER_RATE_PID_INTERVAL_US {2000};	// 500 Hz
constexpr float ATTITUDE_RATE_LIMIT_DPS {180.0f}; // this has been multiplied 180x since last edit, just saying
constexpr int PITCH_PID_OUTPUT_LIMIT {200};
constexpr int YAW_PID_OUTPUT_LIMIT {200};
constexpr int ROLL_PID_OUTPUT_LIMIT {200};
constexpr float INNER_PID_FILTER_ALPHA {0.3f};


constexpr dshot_mode_e DSHOT_TYPE{DSHOT300};

class Motor {
public:
	Motor(MovementController& mc, Drone& drone);
	void begin();
	void loop();
	void Kill();
	void setAttitudePidTunings(const PID& pitch, const PID& roll, const PID& yaw);
	Attitude getCommandOutput() const;
	
private:
	// PID K_pitchAngle{0.4659, 0.2, 0.1};
	// PID K_rollAngle{3, 0.2, 0.6};
	// PID K_yawAngle{2, 0, 1};
	PID K_pitchAngle{0, 0, 0};
	PID K_rollAngle{0, 0, 0};
	PID K_yawAngle{0, 0, 0};

	// PID K_pitchRate{0.4659, 0, 0.1};
	// PID K_rollRate{3, 0.2, 0.6};
	// PID K_yawRate{2, 0, 1};
	PID K_pitchRate{0.4659, 0, 0};
	PID K_rollRate{3, 0.2, 0};
	PID K_yawRate{2, 0, 0};

	MovementController& movementController;
	Drone& drone;
	Attitude target{};
	DShotRMT motor1;
	DShotRMT motor2;
	DShotRMT motor3;
	DShotRMT motor4;

	QuickPID pitchAngle;
	QuickPID rollAngle;
	QuickPID yawAngle;
	QuickPID pitchRate;
	QuickPID rollRate;
	QuickPID yawRate;
	void arm();
	void disarm();
	float pidAuthority;
	Attitude commandOutput;

	// Per-loop attitude snapshots used as direct (unfiltered) QuickPID inputs.
	float pitchInput;
	float yawInput;
	float rollInput;
	float throttleBase;

	float pitchAngle_OUT;
	float yawAngle_OUT;
	float rollAngle_OUT;

	float pitchRate_OUT;
	float yawRate_OUT;
	float rollRate_OUT;

	float pitchRateFiltered_OUT;
	float yawRateFiltered_OUT;
	float rollRateFiltered_OUT;

	uint32_t lastOuterPidComputeUs;
	uint32_t lastInnerPidComputeUs;

	double m1;
	double m2;
	double m3;
	double m4;
};