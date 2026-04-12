#pragma once
#include <Arduino.h>
#include <DShotRMT.h>
#include <QuickPID.h>
#include "MovementController.h"
#include "Status.h"

// CONFIGURATION
constexpr int MOTOR1 {27}; // FRONT RIGHT	CCW
constexpr int MOTOR2 {14}; // BACK RIGHT	CW
constexpr int MOTOR3 {12}; // FRONT LEFT	CW
constexpr int MOTOR4 {13}; // BACK LEFT		CCW

constexpr int INITILIZE_ESC_TIME (40); // 4000
constexpr int MINIMUM_MOTOR_SPEED (70);
constexpr int MAXIMUM_MOTOR_SPEED (1000);
constexpr int PID_MAX_EFFECT_AFTER_SPEED (200);
constexpr int VELOCITY_PID_SAMPLE_US (10000);	// Outer PID loop
constexpr int ATTITUDE_PID_SAMPLE_US (1000);	// Inner PID loop
constexpr int PITCH_PID_OUTPUT_LIMIT (200);
constexpr int YAW_PID_OUTPUT_LIMIT (200);
constexpr int ROLL_PID_OUTPUT_LIMIT (200);
constexpr int VELOCITY_PID_OUTPUT_LIMIT (100);
constexpr float AXIS_INPUT_LPF_ALPHA (0.05f);
constexpr float AXIS_OUTPUT_SLEW_PER_LOOP (12.0f);
constexpr float ATTITUDE_I_BLEED_ZERO_ERROR (3.0f);
constexpr float ATTITUDE_I_BLEED_FULL_ERROR (8.0f);
constexpr float ATTITUDE_I_BLEED_MAX_PER_LOOP (0.01f);
constexpr float VELOCITY_I_BLEED_ZERO_ERROR (0.05f);
constexpr float VELOCITY_I_BLEED_FULL_ERROR (0.50f);
constexpr float VELOCITY_I_BLEED_MAX_PER_LOOP (0.02f);
constexpr float FRONT_BIAS (1.12f);
constexpr float ACCEL_DEADBAND_G (0.02f);
constexpr float VELOCITY_DECAY_PER_SEC (0.1f);

constexpr dshot_mode_e DSHOT_TYPE{DSHOT300};

class Motor {
public:
	Motor(MovementController& mc, Status& s);
	void begin();
	void loop();
	void setAttitudePidTunings(const PID& pitch, const PID& roll, const PID& yaw);
	// D -> 2 for lite, 4 for mye
	// 
	// 
	PID pitchPid{8, 4, 0.7};
	PID yawPid{3, 0, 0.7};
	PID rollPid{5, 4, 0.6};
	PID velocityXPid{5, 0, 30};
	PID velocityYPid{5, 0, 30};

private:
	MovementController& movementController;
	Status& status;
	DShotRMT motor1;
	DShotRMT motor2;
	DShotRMT motor3;
	DShotRMT motor4;
	Attitude target{};

	QuickPID pitchQuickPID;
	QuickPID rollQuickPID;
	QuickPID yawQuickPID;
	QuickPID velocityXQuickPID;
	QuickPID velocityYQuickPID;
	float computeIntegralBleed(float errorAbs, float zeroError, float fullError, float maxBleedPerLoop) const;
	void updateAxisPid(QuickPID& pid, float setpoint, float measurement, float& filteredInput, float& pidOutput, float& command, float outputLimit, float iBleedZeroError, float iBleedFullError, float iBleedMaxPerLoop);
	void updateMotionCorrection(float dtSeconds, float pidAuthority);
	float pidAuthority;


	float pitchInput;
	float yawInput;
	float rollInput;
	float velocityXInput;
	float velocityYInput;
	float xVelocity;
	float yVelocity;
	unsigned long lastMotionUpdateMs;
	unsigned long lastOuterLoopUs;
	unsigned long lastInnerLoopUs;

	float quickPitchOUT;
	float quickYawOUT;
	float quickRollOUT;
	float velocityXOUT;
	float velocityYOUT;

	float quickPitchCommand;
	float quickYawCommand;
	float quickRollCommand;
	float velocityXTarget;
	float velocityYTarget;

	double m1;
	double m2;
	double m3;
	double m4;
};