#pragma once
#include "PID.h"
#include <Arduino.h>
#include "MovementController.h"
#include "Status.h"
#include "DShotRMT.h"

// CONFIGURATION
constexpr int MOTOR1 {27};
constexpr int MOTOR2 {15};
constexpr int MOTOR3 {16};
constexpr int MOTOR4 {17};

#define DSHOT_TYPE DSHOT300

class Motor {
public:
	Motor(MovementController& mc, Status& s);
	void begin();
	void loop();
private:
	MovementController movementController;
	Status& status;
	DShotRMT motor1;
};