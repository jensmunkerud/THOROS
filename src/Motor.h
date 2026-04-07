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

constexpr double Kp_att = 4.0;    // outer loop gain (attitude)
constexpr double Kp_rate = 0.4;   // inner loop proportional
constexpr double Ki_rate = 0.05;
constexpr double Kd_rate = 0.001;
constexpr double dt = 0.1;


constexpr int INITILIZE_ESC_TIME (40); // 4000
constexpr int MINIMUM_MOTOR_SPEED (70);
constexpr int MAXIMUM_MOTOR_SPEED (1000);

constexpr dshot_mode_e DSHOT_TYPE{DSHOT300};

class Motor {
public:
	Motor(MovementController& mc, Status& s);
	void begin();
	void loop();

	PID pitchPid{2, 0, 10};
	PID yawPid{2, 0, 10};
	PID rollPid{2, 0, 10};

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