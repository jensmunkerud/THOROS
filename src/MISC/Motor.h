#pragma once
#include <Arduino.h>
#include <DShotRMT.h>
#include <QuickPID.h>
#include "MovementController.h"
#include "Datatypes.h"

// CONFIGURATION
constexpr int MOTOR1 {32}; // FRONT RIGHT	CCW
constexpr int MOTOR2 {33}; // BACK RIGHT	CW
constexpr int MOTOR3 {25}; // FRONT LEFT	CW
constexpr int MOTOR4 {26}; // BACK LEFT		CCW

constexpr int INITILIZE_ESC_TIME {40}; // 4000
constexpr int MIN_ARMED_DSHOT_VALUE {0}; // 0-47 is dShot reserved values
constexpr int MAXIMUM_MOTOR_SPEED {1200};
constexpr float MOTOR_KILL_SPEED {1500.0f};	// [units/s]
constexpr float MAX_DISARM_TILT_ANGLE_DEG {30.0f};
constexpr float FRONT_BIAS {1.1f};

constexpr int PID_MAX_EFFECT_AFTER_SPEED {200};
constexpr int ATTITUDE_PID_SAMPLE_US {1000};	// Inner PID loop
constexpr int PITCH_PID_OUTPUT_LIMIT {200};
constexpr int YAW_PID_OUTPUT_LIMIT {200};
constexpr int ROLL_PID_OUTPUT_LIMIT {200};
constexpr float AXIS_OUTPUT_SLEW_PER_LOOP {1.0f};
constexpr float ATTITUDE_I_BLEED_ZERO_ERROR {3.0f};
constexpr float ATTITUDE_I_BLEED_FULL_ERROR {8.0f};
constexpr float ATTITUDE_I_BLEED_MAX_PER_LOOP {0.01f};


constexpr dshot_mode_e DSHOT_TYPE{DSHOT300};

class Motor {
public:
	Motor(MovementController& mc, Drone& drone);
	void begin();
	void loop();
	void Kill();
	void setAttitudePidTunings(const PID& pitch, const PID& roll, const PID& yaw);
	Attitude getControlOutput() const;
	
private:
	// PID pitchPid{6, 0.2, 0.7};
	// PID yawPid{2, 4, 1};
	// PID rollPid{3, 0.2, 0.6};
	PID pitchPid{0.4659, 0.2, 0.1};
	PID yawPid{2, 4, 1};
	PID rollPid{3, 0.2, 0.6};
	MovementController& movementController;
	Drone& drone;
	Attitude target{};
	DShotRMT motor1;
	DShotRMT motor2;
	DShotRMT motor3;
	DShotRMT motor4;

	QuickPID pitchQuickPID;
	QuickPID rollQuickPID;
	QuickPID yawQuickPID;
	void arm();
	void disarm();
	float computeIntegralBleed(float errorAbs, float zeroError, float fullError, float maxBleedPerLoop) const;
	void updateAxisPid(QuickPID& pid, float setpoint, float measurement, float& pidOutput, float& command, float outputLimit, float iBleedZeroError, float iBleedFullError, float iBleedMaxPerLoop);
	float pidAuthority;
	Attitude controlOutput;

	// Per-loop attitude snapshots used as direct (unfiltered) QuickPID inputs.
	float pitchInput;
	float yawInput;
	float rollInput;
	float throttleBase;

	float quickPitchOUT;
	float quickYawOUT;
	float quickRollOUT;

	float quickPitchCommand;
	float quickYawCommand;
	float quickRollCommand;

	double m1;
	double m2;
	double m3;
	double m4;
};