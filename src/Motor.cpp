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
yawQuickPID{&yawInput, &quickYawOUT, &target.yaw, yawPid.P, yawPid.I, yawPid.D, QuickPID::Action::direct},
rollQuickPID{&rollInput, &quickRollOUT, &target.roll, rollPid.P, rollPid.I, rollPid.D, QuickPID::Action::reverse},
frontScale{1.0f + FRONT_BIAS},
rearScale{1.0f - FRONT_BIAS}
{}

void Motor::begin() {
	motor1.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor2.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor3.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor4.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);

	pitchQuickPID.SetMode(QuickPID::Control::automatic);
	rollQuickPID.SetMode(QuickPID::Control::automatic);
	yawQuickPID.SetMode(QuickPID::Control::automatic);

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


void Motor::updateAxisPid(QuickPID& pid, float measurement, float& filteredInput, float& pidOutput, float& command, float outputLimit) {
	filteredInput += AXIS_INPUT_LPF_ALPHA * (measurement - filteredInput);
	pid.Compute();
	command = constrain(
		command + constrain(pidOutput - command, -AXIS_OUTPUT_SLEW_PER_LOOP, AXIS_OUTPUT_SLEW_PER_LOOP),
		-outputLimit,
		outputLimit
	);
}


void Motor::loop() {
	if (status.RFD900 != 1) {
		motor1.send_dshot_value(0);
		motor2.send_dshot_value(0);
		motor3.send_dshot_value(0);
		motor4.send_dshot_value(0);
		return;
	}

	updateAxisPid(pitchQuickPID, status.attitude.pitch, pitchInput, quickPitchOUT, quickPitchCommand, PITCH_PID_OUTPUT_LIMIT);
	updateAxisPid(rollQuickPID, status.attitude.roll, rollInput, quickRollOUT, quickRollCommand, ROLL_PID_OUTPUT_LIMIT);
	updateAxisPid(yawQuickPID, status.attitude.yaw, yawInput, quickYawOUT, quickYawCommand, YAW_PID_OUTPUT_LIMIT);
	
	// QuickPID Setup
	float throttleBase = movementController.currentInput.throttle;
	pidAuthority = constrain(
			(throttleBase - (float)MINIMUM_MOTOR_SPEED) / (float)(PID_MAX_EFFECT_AFTER_SPEED - MINIMUM_MOTOR_SPEED),
			0.0f,
			1.0f
	);

	float pitchCmd = quickPitchCommand * pidAuthority;
	float rollCmd = quickRollCommand * pidAuthority;
	float yawCmd = quickYawCommand * pidAuthority;

	m1 = constrain(throttleBase * frontScale + pitchCmd * frontScale - rollCmd + yawCmd * 0, 0, MAXIMUM_MOTOR_SPEED);
	m2 = constrain(throttleBase * rearScale  - pitchCmd * rearScale  - rollCmd - yawCmd * 0, 0, MAXIMUM_MOTOR_SPEED);
	m3 = constrain(throttleBase * frontScale + pitchCmd * frontScale + rollCmd - yawCmd * 0, 0, MAXIMUM_MOTOR_SPEED);
	m4 = constrain(throttleBase * rearScale  - pitchCmd * rearScale  + rollCmd + yawCmd * 0, 0, MAXIMUM_MOTOR_SPEED);

	motor1.send_dshot_value((int)m1);
	motor2.send_dshot_value((int)m2);
	motor3.send_dshot_value((int)m3);
	motor4.send_dshot_value((int)m4);

	return;
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