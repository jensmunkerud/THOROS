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

constexpr double Kp_att = 4.0;    // outer loop gain (attitude)
constexpr double Kp_rate = 0.4;   // inner loop proportional
constexpr double Ki_rate = 0.05;
constexpr double Kd_rate = 0.001;
constexpr double dt = 0.1;


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
	Quaternion target;
	Quaternion error;
	double p_int = 0.0, q_int = 0.0, r_int = 0.0;
	double p_prev = 0.0, q_prev = 0.0, r_prev = 0.0;
};