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
constexpr int AXIS_PID_SAMPLE_US (2000);
constexpr int PITCH_PID_OUTPUT_LIMIT (200);
constexpr int YAW_PID_OUTPUT_LIMIT (200);
constexpr int ROLL_PID_OUTPUT_LIMIT (200);
constexpr float AXIS_INPUT_LPF_ALPHA (0.15f);
constexpr float AXIS_OUTPUT_SLEW_PER_LOOP (12.0f);

constexpr dshot_mode_e DSHOT_TYPE{DSHOT300};

class Motor {
public:
	Motor(MovementController& mc, Status& s);
	void begin();
	void loop();

	PID pitchPid{5, 0, 2};
	PID yawPid{5, 0, 2};
	PID rollPid{5, 0, 2};

private:
	MovementController& movementController;
	Status& status;
	DShotRMT motor1;
	DShotRMT motor2;
	DShotRMT motor3;
	DShotRMT motor4;
	Attitude computePID();
	Attitude target{};
	Attitude resultPID;

	QuickPID pitchQuickPID;
	QuickPID rollQuickPID;
	QuickPID yawQuickPID;

	float pitchInput;
	float yawInput;
	float rollInput;

	float quickPitchOUT;
	float quickYawOUT;
	float quickRollOUT;

	float quickPitchCommand;
	float quickYawCommand;
	float quickRollCommand;


	// PID constants
	double dt = 0.001;

	double m1;
	double m2;
	double m3;
	double m4;

	// Error tracking
	float errorPitch, errorRoll, errorYaw;
	float integralPitch = 0, integralRoll = 0, integralYaw = 0;
	float lastErrorPitch = 0, lastErrorRoll = 0, lastErrorYaw = 0;
};