#include "Motor.h"

Motor::Motor(MovementController& mc, Status& s) : 
movementController{mc},
status{s},
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
rollQuickPID{&rollInput, &quickRollOUT, &target.roll, rollPid.P, rollPid.I, rollPid.D, QuickPID::Action::reverse}
{}

void Motor::begin() {
	motor1.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor2.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor3.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor4.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);

	pitchQuickPID.SetMode(QuickPID::Control::automatic);
	rollQuickPID.SetMode(QuickPID::Control::automatic);
	yawQuickPID.SetMode(QuickPID::Control::automatic);

	pitchQuickPID.SetSampleTimeUs(ATTITUDE_PID_SAMPLE_US);
	rollQuickPID.SetSampleTimeUs(ATTITUDE_PID_SAMPLE_US);
	yawQuickPID.SetSampleTimeUs(ATTITUDE_PID_SAMPLE_US);

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
	status.motorArmed = 1;
}

void Motor::setAttitudePidTunings(const PID& pitch, const PID& roll, const PID& yaw) {
	pitchPid = pitch;
	rollPid = roll;
	yawPid = yaw;
	pitchQuickPID.SetTunings(pitchPid.P, pitchPid.I, pitchPid.D);
	rollQuickPID.SetTunings(rollPid.P, rollPid.I, rollPid.D);
	yawQuickPID.SetTunings(yawPid.P, yawPid.I, yawPid.D);
}


// Perform PID calculation, with slew limiter on output
float Motor::computeIntegralBleed(float errorAbs, float zeroError, float fullError, float maxBleedPerLoop) const {
	if (maxBleedPerLoop <= 0.0f) {
		return 0.0f;
	}
	if (fullError <= zeroError) {
		return maxBleedPerLoop;
	}
	if (errorAbs <= zeroError) {
		return maxBleedPerLoop;
	}
	if (errorAbs >= fullError) {
		return 0.0f;
	}
	float x = (errorAbs - zeroError) / (fullError - zeroError);
	float smooth = x * x * (3.0f - 2.0f * x);
	return maxBleedPerLoop * (1.0f - smooth);
}

void Motor::updateAxisPid(QuickPID& pid, float setpoint, float measuredRaw, float& measuredFiltered, float& commandUnfiltered, float& commandFiltered, float outputLimit, float iBleedZeroError, float iBleedFullError, float iBleedMaxPerLoop) {
	measuredFiltered += AXIS_INPUT_LPF_ALPHA * (measuredRaw - measuredFiltered);
	float errorAbs = fabsf(setpoint - measuredFiltered);
	pid.Compute();

	float iBleed = computeIntegralBleed(errorAbs, iBleedZeroError, iBleedFullError, iBleedMaxPerLoop);
	if (iBleed > 0.0f) {
		pid.SetOutputSum(pid.GetOutputSum() * (1.0f - iBleed));
	}

	commandFiltered = constrain(
		commandFiltered + constrain(commandUnfiltered - commandFiltered, -AXIS_OUTPUT_SLEW_PER_LOOP, AXIS_OUTPUT_SLEW_PER_LOOP),
		-outputLimit,
		outputLimit
	);
}


void Motor::loop() {
	if (status.Communication != 1) {
		motor1.send_dshot_value(0);
		motor2.send_dshot_value(0);
		motor3.send_dshot_value(0);
		motor4.send_dshot_value(0);
		return;
	}

	const ControlInput controlInput = movementController.currentInput;
	target.pitch = controlInput.pitch * ATTITUDE_COMMAND_SCALE;
	target.roll = controlInput.roll * ATTITUDE_COMMAND_SCALE;
	target.yaw = controlInput.yaw * ATTITUDE_COMMAND_SCALE;
	
	float throttleBase = controlInput.throttle;
	pidAuthority = constrain(
			(throttleBase - (float)MINIMUM_MOTOR_SPEED) / (float)(PID_MAX_EFFECT_AFTER_SPEED - MINIMUM_MOTOR_SPEED),
			0.0f,
			1.0f
	);

	// Attitude stabilization
	updateAxisPid(pitchQuickPID, target.pitch, status.attitude.pitch, pitchInput, quickPitchOUT, quickPitchCommand, PITCH_PID_OUTPUT_LIMIT, ATTITUDE_I_BLEED_ZERO_ERROR, ATTITUDE_I_BLEED_FULL_ERROR, ATTITUDE_I_BLEED_MAX_PER_LOOP);
	updateAxisPid(rollQuickPID, target.roll, status.attitude.roll, rollInput, quickRollOUT, quickRollCommand, ROLL_PID_OUTPUT_LIMIT, ATTITUDE_I_BLEED_ZERO_ERROR, ATTITUDE_I_BLEED_FULL_ERROR, ATTITUDE_I_BLEED_MAX_PER_LOOP);
	updateAxisPid(yawQuickPID, target.yaw, status.attitude.yaw, yawInput, quickYawOUT, quickYawCommand, YAW_PID_OUTPUT_LIMIT, ATTITUDE_I_BLEED_ZERO_ERROR, ATTITUDE_I_BLEED_FULL_ERROR, ATTITUDE_I_BLEED_MAX_PER_LOOP);

	// Scale attitude commands by PID authority
	float pitchCmd = quickPitchCommand * pidAuthority;
	float rollCmd = quickRollCommand * pidAuthority;
	float yawCmd = quickYawCommand * pidAuthority;

	// Apply motor mix
	m1 = constrain(throttleBase * FRONT_BIAS + pitchCmd - rollCmd - yawCmd, 0, MAXIMUM_MOTOR_SPEED);
	m2 = constrain(throttleBase              - pitchCmd - rollCmd + yawCmd, 0, MAXIMUM_MOTOR_SPEED);
	m3 = constrain(throttleBase * FRONT_BIAS + pitchCmd + rollCmd + yawCmd, 0, MAXIMUM_MOTOR_SPEED);
	m4 = constrain(throttleBase              - pitchCmd + rollCmd - yawCmd, 0, MAXIMUM_MOTOR_SPEED);

	motor1.send_dshot_value((int)m1);
	motor2.send_dshot_value((int)m2);
	motor3.send_dshot_value((int)m3);
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