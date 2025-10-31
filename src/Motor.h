#pragma once
#include "PID.h"
#include <Arduino.h>
#include "MovementController.h"
#include "Status.h"
#include "Servo.h"

// CONFIGURATION
constexpr int MOTOR1 {14};
constexpr int MOTOR2 {15};
constexpr int MOTOR3 {16};
constexpr int MOTOR4 {17};

class Motor {
public:
	Motor(MovementController& mc, Status& s);
	void loop();
private:
	MovementController movementController;
	Status& status;
	Servo esc1, esc2, esc3, esc4;
};