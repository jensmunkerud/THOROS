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

constexpr int INITILIZE_ESC_TIME (4000); // 4000
constexpr int MINIMUM_MOTOR_SPEED (70);
constexpr int MAXIMUM_MOTOR_SPEED (1000);
constexpr int PID_MAX_EFFECT_AFTER_SPEED (200);
constexpr int AXIS_PID_SAMPLE_US (1000);
constexpr int PITCH_PID_OUTPUT_LIMIT (200);
constexpr int YAW_PID_OUTPUT_LIMIT (200);
constexpr int ROLL_PID_OUTPUT_LIMIT (200);
constexpr float AXIS_INPUT_LPF_ALPHA (0.15f);
constexpr float AXIS_OUTPUT_SLEW_PER_LOOP (12.0f);
constexpr float FRONT_BIAS = 0.1f;

constexpr dshot_mode_e DSHOT_TYPE{DSHOT300};

class Motor {
public:
	Motor(MovementController& mc, Status& s);
	void begin();
	void loop();

	PID pitchPid{8, 1, 1};
	PID yawPid{8, 1, 1};
	PID rollPid{8, 1, 1};

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
	void updateAxisPid(QuickPID& pid, float measurement, float& filteredInput, float& pidOutput, float& command, float outputLimit);
	float pidAuthority;

	float frontScale;
	float rearScale;

	float pitchInput;
	float yawInput;
	float rollInput;

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