#pragma once
#include "PID.h"
#include <Arduino.h>
#include "MovementController.h"
#include "Status.h"
#include "DShotRMT.h"

// CONFIGURATION
constexpr int MOTOR1 {27};
constexpr int MOTOR2 {14};
constexpr int MOTOR3 {12};
constexpr int MOTOR4 {13};

// we can ABSOLUTELY NOT send negative DShot values!!

constexpr int speed {100};	// Vehicle speed
constexpr int P {1};		// Proportional parameter
constexpr int I {1};		// Integral parameter
constexpr int D {1};		// Derivative parameter

constexpr int INITILIZE_ESC_TIME {4000};
constexpr int MINIMUM_MOTOR_SPEED(70);

constexpr dshot_mode_e DSHOT_TYPE{DSHOT300};

class Motor {
public:
	Motor(MovementController& mc, Status& s);
	void begin();
	void loop();
private:
	MovementController& movementController;
	Status& status;
	DShotRMT motor1;
	DShotRMT motor2;
	DShotRMT motor3;
	DShotRMT motor4;
};