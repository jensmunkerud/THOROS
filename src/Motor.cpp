#include "Motor.h"

Motor::Motor(MovementController& mc, Status& s) : movementController{mc}, status{s}, 
motor1(MOTOR1),
motor2(MOTOR2),
motor3(MOTOR3),
motor4(MOTOR4),
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


Attitude Motor::computePID() {
	// Errors
	errorPitch = target.pitch - status.attitude.pitch;
	errorRoll  = target.roll  - status.attitude.roll;
	errorYaw   = target.yaw   - status.attitude.yaw;

	// Integrals
	integralPitch += errorPitch * dt;
	integralRoll  += errorRoll * dt;
	integralYaw   += errorYaw * dt;

	// derivatives
	float dPitch = (errorPitch - lastErrorPitch) / dt;
	float dRoll  = (errorRoll  - lastErrorRoll)  / dt;
	float dYaw   = (errorYaw   - lastErrorYaw)   / dt;

	// PID outputs
	float pidPitch = pitchPid.P * errorPitch + pitchPid.I * integralPitch + pitchPid.D * dPitch;
	float pidRoll  = rollPid.P * errorRoll  + rollPid.I * integralRoll  + rollPid.D * dRoll;
	float pidYaw   = yawPid.P * errorYaw   + yawPid.I * integralYaw   + yawPid.D * dYaw;

	// Save last errors
	lastErrorPitch = errorPitch;
	lastErrorRoll  = errorRoll;
	lastErrorYaw   = errorYaw;
	// Serial.println(errorPitch);
	
	// Mix to motors
	return {pidPitch, pidRoll, pidYaw};
}


void Motor::loop() {
	if (status.RFD900 != 0) {
		motor1.send_dshot_value(0);
		motor2.send_dshot_value(0);
		motor3.send_dshot_value(0);
		motor4.send_dshot_value(0);
		return;
	}

	resultPID = computePID();

	m1 = 0;
	m2 = 0;
	m3 = 0;
	m4 = 0;
	// Serial.println(movementController.currentInput.throttle);

	m1 = constrain(movementController.currentInput.throttle - resultPID.roll * 0 - resultPID.pitch + resultPID.yaw * 0, MINIMUM_MOTOR_SPEED, MAXIMUM_MOTOR_SPEED);
	m2 = constrain(movementController.currentInput.throttle - resultPID.roll * 0 + resultPID.pitch - resultPID.yaw * 0, MINIMUM_MOTOR_SPEED, MAXIMUM_MOTOR_SPEED);
	m3 = constrain(movementController.currentInput.throttle + resultPID.roll * 0 - resultPID.pitch - resultPID.yaw * 0, MINIMUM_MOTOR_SPEED, MAXIMUM_MOTOR_SPEED);
	m4 = constrain(movementController.currentInput.throttle + resultPID.roll * 0 + resultPID.pitch + resultPID.yaw * 0, MINIMUM_MOTOR_SPEED, MAXIMUM_MOTOR_SPEED);

	motor1.send_dshot_value((int)m1);
	motor2.send_dshot_value((int)m2);
	motor3.send_dshot_value((int)m3); // PROPELLER
	motor4.send_dshot_value((int)m4);

	Serial.print(status.attitude.pitch);
	Serial.print("/");
	Serial.print(status.attitude.yaw);
	Serial.print("/");
	Serial.print(status.attitude.roll);
	Serial.print("/");
	Serial.print((int)m1);
	Serial.print("/");
	Serial.print((int)m2);
	Serial.print("/");
	Serial.print((int)m3);
	Serial.print("/");
	Serial.println((int)m4);
}