#include "Motor.h"

Motor::Motor(MovementController& mc, Status& s) : movementController{mc}, status{s}, 
motor1(MOTOR1),
motor2(MOTOR2),
motor3(MOTOR3),
motor4(MOTOR4),
targetPitch{0},
targetYaw{0},
targetRoll{0},
m1{0},
m2{0},
m3{0},
m4{0}
{}

void Motor::begin() {
	motor1.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor2.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor3.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor4.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);

	for(int i = 0; i < INITILIZE_ESC_TIME; i++) {
		motor1.send_dshot_value(0);
		motor2.send_dshot_value(0);
		motor3.send_dshot_value(0);
		motor4.send_dshot_value(0);
		delay(1);
	}
}


Orientation Motor::computePID() {
	Kp = movementController.Kp;
	Ki = movementController.Ki;
	Kd = movementController.Kd;

	status.P = Kp;
	status.I = Ki;
	status.D = Kd;

	// Errors
	errorPitch = targetPitch - status.attitude.pitch;
	errorRoll  = targetRoll  - status.attitude.roll;
	errorYaw   = targetYaw   - status.attitude.yaw;

	// Integrals
	integralPitch += errorPitch * dt;
	integralRoll  += errorRoll * dt;
	integralYaw   += errorYaw * dt;

	// derivatives
	double dPitch = (errorPitch - lastErrorPitch) / dt;
	double dRoll  = (errorRoll  - lastErrorRoll)  / dt;
	double dYaw   = (errorYaw   - lastErrorYaw)   / dt;

	// PID outputs
	double pidPitch = Kp * errorPitch + Ki * integralPitch + Kd * dPitch;
	double pidRoll  = Kp * errorRoll  + Ki * integralRoll  + Kd * dRoll;
	double pidYaw   = Kp * errorYaw   + Ki * integralYaw   + Kd * dYaw;

	// Save last errors
	lastErrorPitch = errorPitch;
	lastErrorRoll  = errorRoll;
	lastErrorYaw   = errorYaw;
	
	// Mix to motors
	return {pidPitch, pidRoll, pidYaw};
}


void Motor::loop() {
	if (status.RFD900 != 1) {
		motor1.send_dshot_value(0);
		motor2.send_dshot_value(0);
		motor3.send_dshot_value(0);
		motor4.send_dshot_value(0);
		return;
	}

	// pid = computePID();
	// m4 = status.speed + MINIMUM_MOTOR_SPEED + pid.pitch + pid.roll - pid.yaw; // Front Right
	// m3 = status.speed + MINIMUM_MOTOR_SPEED - pid.pitch - pid.roll - pid.yaw; // Rear Left
	// m2 = movementController.getInput().pitch + MINIMUM_MOTOR_SPEED + pid.pitch - pid.roll + pid.yaw; // Front Left
	// m1 = status.speed + MINIMUM_MOTOR_SPEED - pid.pitch + pid.roll + pid.yaw; // Rear Right

	m1 = 0;
	m2 = 0;
	m3 = 0;
	m4 = 0;
	// Serial.println(movementController.currentInput.throttle);

	// m2 = movementController.currentInput.throttle;
	// m4 = movementController.currentInput.throttle;
	m1 = movementController.currentInput.throttle;
	m3 = movementController.currentInput.throttle;

	m1 = constrain(m1, 0, 500);
	m2 = constrain(m2, 0, 1000);
	m3 = constrain(m3, 0, 500);
	m4 = constrain(m4, 0, 1000);

	motor1.send_dshot_value((int)m1);
	motor2.send_dshot_value((int)m2);
	motor3.send_dshot_value((int)m3); // PROPELLER
	motor4.send_dshot_value((int)m4);
	// Serial.print((int)m1);
	// Serial.print("\t\t|\t");
	// Serial.print((int)m2);
	// Serial.print("\t\t|\t");
	// Serial.print((int)m3);
	// Serial.print("\t\t|\t");
	// Serial.println((int)m4);
}