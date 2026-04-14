#include "Motor.h"

static inline float degToRad(float deg) {
	return deg * DEG_TO_RAD;
}

static void bodyAccelToWorldHorizontal(float axBody, float ayBody, float azBody, float rollDeg, float pitchDeg, float yawDeg, float& axWorld, float& ayWorld) {
	const float roll = degToRad(rollDeg);
	const float pitch = degToRad(pitchDeg);
	const float yaw = degToRad(yawDeg);

	const float cr = cosf(roll);
	const float sr = sinf(roll);
	const float cp = cosf(pitch);
	const float sp = sinf(pitch);
	const float cy = cosf(yaw);
	const float sy = sinf(yaw);

	// R_b2w = Rz(yaw) * Ry(pitch) * Rx(roll)
	axWorld = (cy * cp) * axBody + (cy * sp * sr - sy * cr) * ayBody + (cy * sp * cr + sy * sr) * azBody;
	ayWorld = (sy * cp) * axBody + (sy * sp * sr + cy * cr) * ayBody + (sy * sp * cr - cy * sr) * azBody;
}

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
velocityXInput{0},
velocityYInput{0},
quickPitchOUT{0},
quickRollOUT{0},
quickYawOUT{0},
velocityXOUT{0},
velocityYOUT{0},
quickPitchCommand{0},
quickRollCommand{0},
quickYawCommand{0},
velocityXTarget{0},
velocityYTarget{0},
pitchQuickPID{&pitchInput, &quickPitchOUT, &target.pitch, pitchPid.P, pitchPid.I, pitchPid.D, QuickPID::Action::direct},
yawQuickPID{&yawInput, &quickYawOUT, &target.yaw, yawPid.P, yawPid.I, yawPid.D, QuickPID::Action::direct},
rollQuickPID{&rollInput, &quickRollOUT, &target.roll, rollPid.P, rollPid.I, rollPid.D, QuickPID::Action::reverse},
velocityXQuickPID{&velocityXInput, &velocityXOUT, &velocityXTarget, velocityXPid.P, velocityXPid.I, velocityXPid.D, QuickPID::Action::direct},
velocityYQuickPID{&velocityYInput, &velocityYOUT, &velocityYTarget, velocityYPid.P, velocityYPid.I, velocityYPid.D, QuickPID::Action::direct},
accelX{0},
accelY{0},
worldVelocityX{0},
worldVelocityY{0},
xVelocity{0},
yVelocity{0},
lastMotionUpdateMs{0},
lastOuterLoopUs{0},
lastInnerLoopUs{0}
{}

void Motor::begin() {
	motor1.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor2.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor3.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor4.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);

	pitchQuickPID.SetMode(QuickPID::Control::automatic);
	rollQuickPID.SetMode(QuickPID::Control::automatic);
	yawQuickPID.SetMode(QuickPID::Control::automatic);
	velocityXQuickPID.SetMode(QuickPID::Control::automatic);
	velocityYQuickPID.SetMode(QuickPID::Control::automatic);

	pitchQuickPID.SetSampleTimeUs(ATTITUDE_PID_SAMPLE_US);
	rollQuickPID.SetSampleTimeUs(ATTITUDE_PID_SAMPLE_US);
	yawQuickPID.SetSampleTimeUs(ATTITUDE_PID_SAMPLE_US);
	velocityXQuickPID.SetSampleTimeUs(VELOCITY_PID_SAMPLE_US);
	velocityYQuickPID.SetSampleTimeUs(VELOCITY_PID_SAMPLE_US);

	pitchQuickPID.SetOutputLimits(-PITCH_PID_OUTPUT_LIMIT, PITCH_PID_OUTPUT_LIMIT);
	rollQuickPID.SetOutputLimits(-ROLL_PID_OUTPUT_LIMIT, ROLL_PID_OUTPUT_LIMIT);
	yawQuickPID.SetOutputLimits(-YAW_PID_OUTPUT_LIMIT, YAW_PID_OUTPUT_LIMIT);
	velocityXQuickPID.SetOutputLimits(-VELOCITY_PID_OUTPUT_LIMIT, VELOCITY_PID_OUTPUT_LIMIT);
	velocityYQuickPID.SetOutputLimits(-VELOCITY_PID_OUTPUT_LIMIT, VELOCITY_PID_OUTPUT_LIMIT);
	lastMotionUpdateMs = millis();
	lastOuterLoopUs = micros();
	lastInnerLoopUs = lastOuterLoopUs;
	
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


// Estimate x y velocity from x y acceleration (Drone space)
void Motor::updateMotionCorrection(float dtSeconds, float pidAuthority) {
	if (dtSeconds <= 0.0f) {
		return;
	}

	accelX = status.linearAccel.x;
	accelY = status.linearAccel.y;

	if (fabsf(accelX) < ACCEL_DEADBAND_G) {
		accelX = 0.0f;
	}
	if (fabsf(accelY) < ACCEL_DEADBAND_G) {
		accelY = 0.0f;
	}

	// Integrate horizontal acceleration in world frame, then project back to body frame.
	float axWorld = 0.0f;
	float ayWorld = 0.0f;
	bodyAccelToWorldHorizontal(
		accelX,
		accelY,
		accelRawZ,
		status.attitude.roll,
		status.attitude.pitch,
		status.attitude.yaw,
		axWorld,
		ayWorld
	);

	worldVelocityX += axWorld * 9.81f * dt;
	worldVelocityY += ayWorld * 9.81f * dt;

	// Apply velocity decay to prevent drift
	float decay = constrain(1.0f - VELOCITY_DECAY_PER_SEC * dt, 0.0f, 1.0f);
	worldVelocityX *= decay;
	worldVelocityY *= decay;

	const float yaw = degToRad(status.attitude.yaw);
	const float cy = cosf(yaw);
	const float sy = sinf(yaw);
	xVelocity = cy * worldVelocityX + sy * worldVelocityY;
	yVelocity = -sy * worldVelocityX + cy * worldVelocityY;

	if (fabsf(xVelocity) < VELOCITY_ZERO_CLAMP_MPS) {
		xVelocity = 0.0f;
	}
	if (fabsf(yVelocity) < VELOCITY_ZERO_CLAMP_MPS) {
		yVelocity = 0.0f;
	}
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
	// if (status.Communication != 1) {
	// 	xVelocity = 0.0f;
	// 	yVelocity = 0.0f;
	// 	lastMotionUpdateMs = millis();
	// 	lastOuterLoopUs = micros();
	// 	lastInnerLoopUs = lastOuterLoopUs;
	// 	motor1.send_dshot_value(0);
	// 	motor2.send_dshot_value(0);
	// 	motor3.send_dshot_value(0);
	// 	motor4.send_dshot_value(0);
	// 	return;
	// }

	unsigned long nowUs = micros();
	bool runOuter = (lastOuterLoopUs == 0) || ((unsigned long)(nowUs - lastOuterLoopUs) >= (unsigned long)VELOCITY_PID_SAMPLE_US);
	bool runInner = (lastInnerLoopUs == 0) || ((unsigned long)(nowUs - lastInnerLoopUs) >= (unsigned long)ATTITUDE_PID_SAMPLE_US);

	float dtSeconds = 0.0f;
	if (runOuter) {
		unsigned long elapsedOuterUs = (lastOuterLoopUs == 0) ? 0 : (unsigned long)(nowUs - lastOuterLoopUs);
		dtSeconds = elapsedOuterUs / 1000000.0f;
		lastOuterLoopUs = nowUs;
		lastMotionUpdateMs = millis();
	}
	
	float throttleBase = movementController.currentInput.throttle + 350;
	pidAuthority = constrain(
			(throttleBase - (float)MINIMUM_MOTOR_SPEED) / (float)(PID_MAX_EFFECT_AFTER_SPEED - MINIMUM_MOTOR_SPEED),
			0.0f,
			1.0f
	);

	if (runOuter) {
		// OUTER LOOP: Velocity Control
		// Integrate acceleration to estimate velocity
		updateMotionCorrection(dtSeconds, pidAuthority);

		// Update velocity PIDs - these output target pitch and roll angles
		// velocityXOUT controls the target roll for X velocity
		// velocityYOUT controls the target pitch for Y velocity
		updateAxisPid(velocityXQuickPID, velocityXTarget, xVelocity, velocityXInput, velocityXOUT, target.roll, VELOCITY_PID_OUTPUT_LIMIT, VELOCITY_I_BLEED_ZERO_ERROR, VELOCITY_I_BLEED_FULL_ERROR, VELOCITY_I_BLEED_MAX_PER_LOOP);
		updateAxisPid(velocityYQuickPID, velocityYTarget, yVelocity, velocityYInput, velocityYOUT, target.pitch, VELOCITY_PID_OUTPUT_LIMIT, VELOCITY_I_BLEED_ZERO_ERROR, VELOCITY_I_BLEED_FULL_ERROR, VELOCITY_I_BLEED_MAX_PER_LOOP);
	}

	if (runInner) {
		lastInnerLoopUs = nowUs;
		// INNER LOOP: Attitude Stabilization
		// Filter and control attitude to the targets set by velocity PIDs
		updateAxisPid(pitchQuickPID, target.pitch, status.attitude.pitch, pitchInput, quickPitchOUT, quickPitchCommand, PITCH_PID_OUTPUT_LIMIT, ATTITUDE_I_BLEED_ZERO_ERROR, ATTITUDE_I_BLEED_FULL_ERROR, ATTITUDE_I_BLEED_MAX_PER_LOOP);
		updateAxisPid(rollQuickPID, target.roll, status.attitude.roll, rollInput, quickRollOUT, quickRollCommand, ROLL_PID_OUTPUT_LIMIT, ATTITUDE_I_BLEED_ZERO_ERROR, ATTITUDE_I_BLEED_FULL_ERROR, ATTITUDE_I_BLEED_MAX_PER_LOOP);
		updateAxisPid(yawQuickPID, target.yaw, status.attitude.yaw, yawInput, quickYawOUT, quickYawCommand, YAW_PID_OUTPUT_LIMIT, ATTITUDE_I_BLEED_ZERO_ERROR, ATTITUDE_I_BLEED_FULL_ERROR, ATTITUDE_I_BLEED_MAX_PER_LOOP);
	}

	// Scale attitude commands by PID authority
	float pitchCmd = quickPitchCommand * pidAuthority;
	float rollCmd = quickRollCommand * pidAuthority;
	float yawCmd = quickYawCommand * pidAuthority;

	// Apply only the velocity loop mix (outer loop output only)
	// float pitchCmd = target.pitch * pidAuthority;
	// float rollCmd = - target.roll * pidAuthority;
	// float yawCmd = 0.0f;
	
	// Apply motor mix
	m1 = constrain(throttleBase * FRONT_BIAS + pitchCmd - rollCmd - yawCmd, 0, MAXIMUM_MOTOR_SPEED);
	m2 = constrain(throttleBase              - pitchCmd - rollCmd + yawCmd, 0, MAXIMUM_MOTOR_SPEED);
	m3 = constrain(throttleBase * FRONT_BIAS + pitchCmd + rollCmd + yawCmd, 0, MAXIMUM_MOTOR_SPEED);
	m4 = constrain(throttleBase              - pitchCmd + rollCmd - yawCmd, 0, MAXIMUM_MOTOR_SPEED);

	motor1.send_dshot_value((int)m1);
	motor2.send_dshot_value((int)m2);
	motor3.send_dshot_value((int)m3);
	motor4.send_dshot_value((int)m4);
	
	Serial.println(xVelocity, 8);
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