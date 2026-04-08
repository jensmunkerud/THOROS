#include "Motor.h"

Motor::Motor(MovementController& mc, Status& s) : movementController{mc}, status{s}, 
motor1(MOTOR1),
motor2(MOTOR2),
motor3(MOTOR3),
motor4(MOTOR4),
m1{0},
m2{0},
m3{0},
m4{0},
pitchInput{0},
rollInput{0},
yawInput{0},
quickPitchOUT{0},
quickRollOUT{0},
quickYawOUT{0},
quickPitchCommand{0},
quickRollCommand{0},
quickYawCommand{0},
pitchQuickPID{&pitchInput, &quickPitchOUT, &target.pitch, pitchPid.P, pitchPid.I, pitchPid.D, QuickPID::Action::direct},
rollQuickPID{&rollInput, &quickRollOUT, &target.roll, rollPid.P, rollPid.I, rollPid.D, QuickPID::Action::direct},
yawQuickPID{&yawInput, &quickYawOUT, &target.yaw, yawPid.P, yawPid.I, yawPid.D, QuickPID::Action::direct}
{}

void Motor::begin() {
	motor1.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor2.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor3.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor4.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);

	pitchQuickPID.SetMode(QuickPID::Control::timer);
	rollQuickPID.SetMode(QuickPID::Control::timer);
	yawQuickPID.SetMode(QuickPID::Control::timer);

	pitchQuickPID.SetSampleTimeUs(AXIS_PID_SAMPLE_US);
	rollQuickPID.SetSampleTimeUs(AXIS_PID_SAMPLE_US);
	yawQuickPID.SetSampleTimeUs(AXIS_PID_SAMPLE_US);

	pitchQuickPID.SetOutputLimits(-PITCH_PID_OUTPUT_LIMIT, PITCH_PID_OUTPUT_LIMIT);
	rollQuickPID.SetOutputLimits(-ROLL_PID_OUTPUT_LIMIT, ROLL_PID_OUTPUT_LIMIT);
	yawQuickPID.SetOutputLimits(-YAW_PID_OUTPUT_LIMIT, YAW_PID_OUTPUT_LIMIT);
	
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
	return {pidPitch, pidYaw, pidRoll};
}


void Motor::loop() {
	if (status.RFD900 != 0) {
		motor1.send_dshot_value(0);
		motor2.send_dshot_value(0);
		motor3.send_dshot_value(0);
		motor4.send_dshot_value(0);
		return;
	}

	// resultPID = computePID();
	pitchInput += AXIS_INPUT_LPF_ALPHA * (status.attitude.pitch - pitchInput);
	yawInput += AXIS_INPUT_LPF_ALPHA * (status.attitude.yaw - yawInput);
	rollInput += AXIS_INPUT_LPF_ALPHA * (status.attitude.roll - rollInput);

	pitchQuickPID.Compute();
	rollQuickPID.Compute();
	yawQuickPID.Compute();

	quickPitchCommand = constrain(
		quickPitchCommand + constrain(quickPitchOUT - quickPitchCommand, -AXIS_OUTPUT_SLEW_PER_LOOP, AXIS_OUTPUT_SLEW_PER_LOOP),
		-PITCH_PID_OUTPUT_LIMIT,
		PITCH_PID_OUTPUT_LIMIT
	);
	quickRollCommand = constrain(
		quickRollCommand + constrain(quickRollOUT - quickRollCommand, -AXIS_OUTPUT_SLEW_PER_LOOP, AXIS_OUTPUT_SLEW_PER_LOOP),
		-ROLL_PID_OUTPUT_LIMIT,
		ROLL_PID_OUTPUT_LIMIT
	);
	quickYawCommand = constrain(
		quickYawCommand + constrain(quickYawOUT - quickYawCommand, -AXIS_OUTPUT_SLEW_PER_LOOP, AXIS_OUTPUT_SLEW_PER_LOOP),
		-YAW_PID_OUTPUT_LIMIT,
		YAW_PID_OUTPUT_LIMIT
	);

	m1 = 0;
	m2 = 0;
	m3 = 0;
	m4 = 0;
	// Serial.println(movementController.currentInput.throttle);

	// My custom PID setup
	// m1 = constrain(movementController.currentInput.throttle - resultPID.roll + resultPID.pitch + resultPID.yaw, MINIMUM_MOTOR_SPEED, MAXIMUM_MOTOR_SPEED);
	// m2 = constrain(movementController.currentInput.throttle - resultPID.roll - resultPID.pitch - resultPID.yaw, MINIMUM_MOTOR_SPEED, MAXIMUM_MOTOR_SPEED);
	// m3 = constrain(movementController.currentInput.throttle + resultPID.roll + resultPID.pitch - resultPID.yaw, MINIMUM_MOTOR_SPEED, MAXIMUM_MOTOR_SPEED);
	// m4 = constrain(movementController.currentInput.throttle + resultPID.roll - resultPID.pitch + resultPID.yaw, MINIMUM_MOTOR_SPEED, MAXIMUM_MOTOR_SPEED);

	// QuickPID Setup
	float throttleBase = movementController.currentInput.throttle + 500;
	m1 = constrain(throttleBase - quickPitchCommand - quickRollCommand + quickYawCommand, 0, MAXIMUM_MOTOR_SPEED);
	m2 = constrain(throttleBase + quickPitchCommand - quickRollCommand - quickYawCommand, 0, MAXIMUM_MOTOR_SPEED);
	m3 = constrain(throttleBase - quickPitchCommand + quickRollCommand - quickYawCommand, 0, MAXIMUM_MOTOR_SPEED);
	m4 = constrain(throttleBase + quickPitchCommand + quickRollCommand + quickYawCommand, 0, MAXIMUM_MOTOR_SPEED);

	motor1.send_dshot_value((int)m1);
	motor2.send_dshot_value((int)m2); // PROPELLER
	motor3.send_dshot_value((int)m3);
	motor4.send_dshot_value((int)m4); // PROPELLER

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
	// Serial.println(errorYaw);
}