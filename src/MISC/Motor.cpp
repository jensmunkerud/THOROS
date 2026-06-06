#include "Motor.h"


Motor::Motor(MovementController& mc, Drone& drone) : 
	movementController{mc},
	drone{drone},
	motor1(MOTOR1),
	motor2(MOTOR2),
	motor3(MOTOR3),
	motor4(MOTOR4),
	m1{0},
	m2{0},
	m3{0},
	m4{0},
	throttleBase{0},
	quickPitchOUT{0},
	quickRollOUT{0},
	quickYawOUT{0},
	quickPitchCommand{0},
	quickRollCommand{0},
	quickYawCommand{0},
	commandOutput{},
	pitchQuickPID{&drone.attitude.pitch, &quickPitchOUT, &target.pitch, pitchPid.P, pitchPid.I, pitchPid.D, QuickPID::Action::direct},
	yawQuickPID{&drone.attitude.yaw, &quickYawOUT, &target.yaw, yawPid.P, yawPid.I, yawPid.D, QuickPID::Action::direct},
	rollQuickPID{&drone.attitude.roll, &quickRollOUT, &target.roll, rollPid.P, rollPid.I, rollPid.D, QuickPID::Action::reverse},
	pitchRatePID{&drone.gyroRate.pitch, &quickPitchCommand, &quickPitchOUT, 1.0f, 0.0f, 0.02f, QuickPID::Action::direct},
	yawRatePID{&drone.gyroRate.yaw, &quickYawCommand, &quickYawOUT, 0.8f, 0.0f, 0.0f, QuickPID::Action::direct},
	rollRatePID{&drone.gyroRate.roll, &quickRollCommand, &quickRollOUT, 1.0f, 0.0f, 0.02f, QuickPID::Action::reverse}
{}


Attitude Motor::getCommandOutput() const {
	return commandOutput;
}


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

	pitchQuickPID.SetOutputLimits(-ATTITUDE_RATE_LIMIT_DPS, ATTITUDE_RATE_LIMIT_DPS);
	rollQuickPID.SetOutputLimits(-ATTITUDE_RATE_LIMIT_DPS, ATTITUDE_RATE_LIMIT_DPS);
	yawQuickPID.SetOutputLimits(-ATTITUDE_RATE_LIMIT_DPS, ATTITUDE_RATE_LIMIT_DPS);

	pitchRatePID.SetMode(QuickPID::Control::automatic);
	rollRatePID.SetMode(QuickPID::Control::automatic);
	yawRatePID.SetMode(QuickPID::Control::automatic);

	pitchRatePID.SetSampleTimeUs(ATTITUDE_PID_SAMPLE_US);
	rollRatePID.SetSampleTimeUs(ATTITUDE_PID_SAMPLE_US);
	yawRatePID.SetSampleTimeUs(ATTITUDE_PID_SAMPLE_US);

	pitchRatePID.SetOutputLimits(-PITCH_PID_OUTPUT_LIMIT, PITCH_PID_OUTPUT_LIMIT);
	rollRatePID.SetOutputLimits(-ROLL_PID_OUTPUT_LIMIT, ROLL_PID_OUTPUT_LIMIT);
	yawRatePID.SetOutputLimits(-YAW_PID_OUTPUT_LIMIT, YAW_PID_OUTPUT_LIMIT);
	
	arm();
	{
		DroneLockGuard droneLock(drone);
		drone.MOTOR_OK = true;
	}
}


void Motor::arm() {
	for(int i = 0; i < INITILIZE_ESC_TIME; i++) {
		motor1.send_dshot_value(0);
		motor2.send_dshot_value(0);
		motor3.send_dshot_value(0);
		motor4.send_dshot_value(0);
		delay(1);
	}
	{
		DroneLockGuard droneLock(drone);
		drone.flightControls = {};
		drone.mode = FlightMode::ARMED;
	}
	commandOutput = {};
}

void Motor::disarm() {
	motor1.send_dshot_value(0);
	motor2.send_dshot_value(0);
	motor3.send_dshot_value(0);
	motor4.send_dshot_value(0);
	{
		DroneLockGuard droneLock(drone);
		drone.mode = FlightMode::DISARMED;
		drone.flightControls = {};
		drone.motorThrusts = {};
	}
	commandOutput = {};
	movementController.clearInputs(true);
}


void Motor::Kill() {
	float throttle = 0.0f;
	{
		DroneLockGuard droneLock(drone);
		if (drone.mode == FlightMode::DISARMED) {
			return;
		}
		drone.flightControls.pitch = 0.0f;
		drone.flightControls.roll = 0.0f;
		drone.flightControls.yaw = 0.0f;
		throttle = drone.flightControls.throttle;
	}
	unsigned long lastUpdateMs = millis();

	while (throttle > 0.0f) {
		bool isDisarmed = false;
		{
			DroneLockGuard droneLock(drone);
			isDisarmed = (drone.mode == FlightMode::DISARMED);
		}
		if (isDisarmed) {
			break;
		}

		unsigned long now = millis();
		float deltaSeconds = static_cast<float>(now - lastUpdateMs) / 1000.0f;
		lastUpdateMs = now;

		throttle -= MOTOR_KILL_SPEED * deltaSeconds;
		if (throttle < 0.0f) {
			throttle = 0.0f;
		}

		{
			DroneLockGuard droneLock(drone);
			drone.flightControls.throttle = throttle;
		}
		throttleBase = throttle;
		m1 = throttle;
		m2 = throttle;
		m3 = throttle;
		m4 = throttle;
		{
			DroneLockGuard droneLock(drone);
			drone.motorThrusts = {
				static_cast<int16_t>(m1),
				static_cast<int16_t>(m2),
				static_cast<int16_t>(m3),
				static_cast<int16_t>(m4)
			};
		}

		motor1.send_dshot_value(constrain((int)m1, MIN_ARMED_DSHOT_VALUE, MAXIMUM_MOTOR_SPEED));
		motor2.send_dshot_value(constrain((int)m2, MIN_ARMED_DSHOT_VALUE, MAXIMUM_MOTOR_SPEED));
		motor3.send_dshot_value(constrain((int)m3, MIN_ARMED_DSHOT_VALUE, MAXIMUM_MOTOR_SPEED));
		motor4.send_dshot_value(constrain((int)m4, MIN_ARMED_DSHOT_VALUE, MAXIMUM_MOTOR_SPEED));
	}

	disarm();
}

void Motor::loop() {
	FlightMode mode;
	FlightControls controlInput;
	Attitude attitudeInput;
	{
		DroneLockGuard droneLock(drone);
		mode = drone.mode;
		controlInput = drone.flightControls;
		attitudeInput = drone.attitude;
	}
	if (mode == FlightMode::DISARMED) {return;}

	if (fabsf(attitudeInput.pitch) > MAX_DISARM_TILT_ANGLE_DEG ||
		fabsf(attitudeInput.roll) > MAX_DISARM_TILT_ANGLE_DEG) {
		disarm();
		return;
	}

	target.pitch = controlInput.pitch;
	target.roll = controlInput.roll;
	target.yaw = controlInput.yaw;
	throttleBase = controlInput.throttle;
	
	pidAuthority = constrain(
			(throttleBase - (float)MIN_ARMED_DSHOT_VALUE) / (float)(PID_MAX_EFFECT_AFTER_SPEED - MIN_ARMED_DSHOT_VALUE),
			0.0f,
			1.0f
	);

	// Angle loop -> desired rate
	pitchQuickPID.Compute();
	rollQuickPID.Compute();
	yawQuickPID.Compute();

	// Rate loop -> motor command
	pitchRatePID.Compute();
	rollRatePID.Compute();
	yawRatePID.Compute();

	// Scale rate-loop commands by PID authority
	float pitchCmd = quickPitchCommand * pidAuthority;
	float rollCmd = quickRollCommand * pidAuthority;
	float yawCmd = quickYawCommand * pidAuthority;
	commandOutput = {
		pitchCmd,
		yawCmd,
		rollCmd
	};

	// Apply motor mix
	m1 = constrain(throttleBase * FRONT_BIAS + pitchCmd - rollCmd - yawCmd, 0, MAXIMUM_MOTOR_SPEED);
	m2 = constrain(throttleBase              - pitchCmd - rollCmd + yawCmd, 0, MAXIMUM_MOTOR_SPEED);
	m3 = constrain(throttleBase * FRONT_BIAS + pitchCmd + rollCmd + yawCmd, 0, MAXIMUM_MOTOR_SPEED);
	m4 = constrain(throttleBase              - pitchCmd + rollCmd - yawCmd, 0, MAXIMUM_MOTOR_SPEED);
	{
		DroneLockGuard droneLock(drone);
		drone.motorThrusts = {
			static_cast<int16_t>(m1),
			static_cast<int16_t>(m2),
			static_cast<int16_t>(m3),
			static_cast<int16_t>(m4)
		};
	}

	motor1.send_dshot_value(constrain((int)m1, MIN_ARMED_DSHOT_VALUE, MAXIMUM_MOTOR_SPEED));
	motor2.send_dshot_value(constrain((int)m2, MIN_ARMED_DSHOT_VALUE, MAXIMUM_MOTOR_SPEED));
	motor3.send_dshot_value(constrain((int)m3, MIN_ARMED_DSHOT_VALUE, MAXIMUM_MOTOR_SPEED));
	motor4.send_dshot_value(constrain((int)m4, MIN_ARMED_DSHOT_VALUE, MAXIMUM_MOTOR_SPEED));
	
	return;
	Serial.print(attitudeInput.pitch);
	Serial.print("/");
	Serial.print(attitudeInput.yaw);
	Serial.print("/");
	Serial.print(attitudeInput.roll);
	Serial.print("/");
	Serial.print((int)m1);
	Serial.print("/");
	Serial.print((int)m2);
	Serial.print("/");
	Serial.print((int)m3);
	Serial.print("/");
	Serial.println((int)m4);
}