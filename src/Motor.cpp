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
}


// Estimate x y velocity from x y acceleration (Drone space)
void Motor::updateMotionCorrection(float dtSeconds, float pidAuthority) {
	if (dtSeconds <= 0.0f) {
		return;
	}

	float accelX = status.linearAccel.x;
	float accelY = status.linearAccel.y;

	if (fabsf(accelX) < ACCEL_DEADBAND_G) {
		accelX = 0.0f;
	}
	if (fabsf(accelY) < ACCEL_DEADBAND_G) {
		accelY = 0.0f;
	}

	// Integrate acceleration to velocity
	xVelocity += accelX * 9.81f * dtSeconds;
	yVelocity += accelY * 9.81f * dtSeconds;

	// Apply velocity decay to prevent drift
	float decay = constrain(1.0f - VELOCITY_DECAY_PER_SEC * dtSeconds, 0.0f, 1.0f);
	xVelocity *= decay;
	yVelocity *= decay;
}


// Perform PID calculation, with slew limiter on output
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
	if (status.Communication != 1) {
		xVelocity = 0.0f;
		yVelocity = 0.0f;
		lastMotionUpdateMs = millis();
		lastOuterLoopUs = micros();
		lastInnerLoopUs = lastOuterLoopUs;
		motor1.send_dshot_value(0);
		motor2.send_dshot_value(0);
		motor3.send_dshot_value(0);
		motor4.send_dshot_value(0);
		return;
	}

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
	
	float throttleBase = movementController.currentInput.throttle;
	pidAuthority = constrain(
			(throttleBase - (float)MINIMUM_MOTOR_SPEED) / (float)(PID_MAX_EFFECT_AFTER_SPEED - MINIMUM_MOTOR_SPEED),
			0.0f,
			1.0f
	);

	if (runOuter) {
		// OUTER LOOP: Velocity Control
		// Integrate acceleration to estimate velocity
		updateMotionCorrection(dtSeconds, pidAuthority);

		// Rotate world-frame velocity into drone body frame so X/Y commands stay
		// relative to drone orientation at any yaw angle.
		// float yawRad = radians(status.attitude.yaw);
		// float bodyVelocityX = xVelocity * cosf(yawRad) + yVelocity * sinf(yawRad);
		// float bodyVelocityY = -xVelocity * sinf(yawRad) + yVelocity * cosf(yawRad);

		// Update velocity PIDs - these output target pitch and roll angles
		// velocityXOUT controls the target roll for X velocity
		// velocityYOUT controls the target pitch for Y velocity
		xVelocity = 0;
		yVelocity = 0;
		updateAxisPid(velocityXQuickPID, xVelocity, velocityXInput, velocityXOUT, target.roll, VELOCITY_PID_OUTPUT_LIMIT);
		updateAxisPid(velocityYQuickPID, yVelocity, velocityYInput, velocityYOUT, target.pitch, VELOCITY_PID_OUTPUT_LIMIT);
	}

	if (runInner) {
		lastInnerLoopUs = nowUs;
		// INNER LOOP: Attitude Stabilization
		// Filter and control attitude to the targets set by velocity PIDs
		updateAxisPid(pitchQuickPID, status.attitude.pitch, pitchInput, quickPitchOUT, quickPitchCommand, PITCH_PID_OUTPUT_LIMIT);
		updateAxisPid(rollQuickPID, status.attitude.roll, rollInput, quickRollOUT, quickRollCommand, ROLL_PID_OUTPUT_LIMIT);
		updateAxisPid(yawQuickPID, status.attitude.yaw, yawInput, quickYawOUT, quickYawCommand, YAW_PID_OUTPUT_LIMIT);
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
	m1 = constrain(throttleBase * FRONT_BIAS + pitchCmd - rollCmd + yawCmd * 0, 0, MAXIMUM_MOTOR_SPEED);
	m2 = constrain(throttleBase              - pitchCmd - rollCmd - yawCmd * 0, 0, MAXIMUM_MOTOR_SPEED);
	m3 = constrain(throttleBase * FRONT_BIAS + pitchCmd + rollCmd - yawCmd * 0, 0, MAXIMUM_MOTOR_SPEED);
	m4 = constrain(throttleBase              - pitchCmd + rollCmd + yawCmd * 0, 0, MAXIMUM_MOTOR_SPEED);

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