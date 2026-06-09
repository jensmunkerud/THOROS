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
	pitchAngle_OUT{0},
	rollAngle_OUT{0},
	yawAngle_OUT{0},
	pitchRate_OUT{0},
	rollRate_OUT{0},
	yawRate_OUT{0},
	pitchRateFiltered_OUT{0},
	rollRateFiltered_OUT{0},
	yawRateFiltered_OUT{0},
	lastOuterPidComputeUs{0},
	lastInnerPidComputeUs{0},
	pitchAngle{&drone.attitude.pitch, &pitchAngle_OUT, &target.pitch, K_pitchAngle.P, K_pitchAngle.I, K_pitchAngle.D, QuickPID::Action::direct},
	rollAngle{&drone.attitude.roll, &rollAngle_OUT, &target.roll, K_rollAngle.P, K_rollAngle.I, K_rollAngle.D, QuickPID::Action::reverse},
	yawAngle{&drone.attitude.yaw, &yawAngle_OUT, &target.yaw, K_yawAngle.P, K_yawAngle.I, K_yawAngle.D, QuickPID::Action::direct},
	pitchRate{&drone.gyroRate.pitch, &pitchRate_OUT, &pitchAngle_OUT, K_pitchRate.P, K_pitchRate.I, K_pitchRate.D, QuickPID::Action::direct},
	rollRate{&drone.gyroRate.roll, &rollRate_OUT, &rollAngle_OUT, K_rollRate.P, K_rollRate.I, K_rollRate.D, QuickPID::Action::reverse},
	yawRate{&drone.gyroRate.yaw, &yawRate_OUT, &yawAngle_OUT, K_yawRate.P, K_yawRate.I, K_yawRate.D, QuickPID::Action::direct}
{}



void Motor::begin() {
	motor1.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor2.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor3.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor4.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);

	pitchAngle.SetMode(QuickPID::Control::automatic);
	rollAngle.SetMode(QuickPID::Control::automatic);
	yawAngle.SetMode(QuickPID::Control::automatic);

	pitchAngle.SetSampleTimeUs(OUTER_PID_INTERVAL_US);
	rollAngle.SetSampleTimeUs(OUTER_PID_INTERVAL_US);
	yawAngle.SetSampleTimeUs(OUTER_PID_INTERVAL_US);

	pitchAngle.SetOutputLimits(-ATTITUDE_RATE_LIMIT_DPS, ATTITUDE_RATE_LIMIT_DPS);
	rollAngle.SetOutputLimits(-ATTITUDE_RATE_LIMIT_DPS, ATTITUDE_RATE_LIMIT_DPS);
	yawAngle.SetOutputLimits(-ATTITUDE_RATE_LIMIT_DPS, ATTITUDE_RATE_LIMIT_DPS);

	pitchRate.SetMode(QuickPID::Control::automatic);
	rollRate.SetMode(QuickPID::Control::automatic);
	yawRate.SetMode(QuickPID::Control::automatic);

	pitchRate.SetSampleTimeUs(INNER_RATE_PID_INTERVAL_US);
	rollRate.SetSampleTimeUs(INNER_RATE_PID_INTERVAL_US);
	yawRate.SetSampleTimeUs(INNER_RATE_PID_INTERVAL_US);

	pitchRate.SetOutputLimits(-PITCH_PID_OUTPUT_LIMIT, PITCH_PID_OUTPUT_LIMIT);
	rollRate.SetOutputLimits(-ROLL_PID_OUTPUT_LIMIT, ROLL_PID_OUTPUT_LIMIT);
	yawRate.SetOutputLimits(-YAW_PID_OUTPUT_LIMIT, YAW_PID_OUTPUT_LIMIT);
	
	arm();
	{
		DroneLockGuard droneLock(drone);
		drone.MOTOR_OK = true;
	}
}


void Motor::arm() {
	for(int i = 0; i < INITIALIZE_ESC_TIME; i++) {
		motor1.send_dshot_value(0);
		motor2.send_dshot_value(0);
		motor3.send_dshot_value(0);
		motor4.send_dshot_value(0);
		delay(1);
	}
	{
		DroneLockGuard droneLock(drone);
		drone.flightControls = {};
		drone.commandOutput = {};
		drone.mode = FlightMode::ARMED;
	}
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
		drone.commandOutput = {};
	}
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

void Motor::setAttitudePidTunings(const PID& pitch, const PID& roll, const PID& yaw) {
	pitchRate.SetTunings(pitch.P, pitch.I, pitch.D);
	rollRate.SetTunings(roll.P, roll.I, roll.D);
	yawRate.SetTunings(yaw.P, yaw.I, yaw.D);
}

void Motor::loop() {
	FlightMode mode;
	FlightControls controlInput;
	Attitude attitude;
	{
		DroneLockGuard droneLock(drone);
		mode = drone.mode;
		controlInput = drone.flightControls;
		attitude = drone.attitude;
	}
	if (mode == FlightMode::DISARMED) {return;}

	if (fabsf(attitude.pitch) > MAX_DISARM_TILT_ANGLE_DEG ||
		fabsf(attitude.roll) > MAX_DISARM_TILT_ANGLE_DEG) {
		disarm();
		return;
	}

	target.pitch = controlInput.pitch;
	target.roll = controlInput.roll;
	target.yaw = controlInput.yaw;
	throttleBase = controlInput.throttle;
	uint32_t nowUs = micros();
	bool shouldRunOuter = (uint32_t)(nowUs - lastOuterPidComputeUs) >= OUTER_PID_INTERVAL_US;
	bool shouldRunInner = (uint32_t)(nowUs - lastInnerPidComputeUs) >= INNER_RATE_PID_INTERVAL_US;
	
	pidAuthority = constrain(
			(throttleBase - (float)MIN_ARMED_DSHOT_VALUE) / (float)(PID_MAX_EFFECT_AFTER_SPEED - MIN_ARMED_DSHOT_VALUE),
			0.0f,
			1.0f
	);

	// Angle loop -> desired rate
	if (shouldRunOuter) {
		lastOuterPidComputeUs = nowUs;
		pitchAngle.Compute();
		rollAngle.Compute();
		yawAngle.Compute();
	}

	// Rate loop -> motor command
	if (shouldRunInner) {
		lastInnerPidComputeUs = nowUs;
		pitchRate.Compute();
		rollRate.Compute();
		yawRate.Compute();
		pitchRateFiltered_OUT = Filters::LowPass(pitchRateFiltered_OUT, pitchRate_OUT, INNER_PID_FILTER_ALPHA);
		rollRateFiltered_OUT  = Filters::LowPass(rollRateFiltered_OUT,  rollRate_OUT,  INNER_PID_FILTER_ALPHA);
		yawRateFiltered_OUT   = Filters::LowPass(yawRateFiltered_OUT,   yawRate_OUT,   INNER_PID_FILTER_ALPHA);
	}

	// Scale rate-loop commands by PID authority
	float pitchCmd = pitchRateFiltered_OUT * pidAuthority;
	float rollCmd = rollRateFiltered_OUT * pidAuthority;
	float yawCmd = yawRateFiltered_OUT * pidAuthority;
	commandOutput = { pitchCmd, yawCmd, rollCmd };

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
		drone.commandOutput = commandOutput;
	}

	motor1.send_dshot_value(constrain((int)m1, MIN_ARMED_DSHOT_VALUE, MAXIMUM_MOTOR_SPEED));
	motor2.send_dshot_value(constrain((int)m2, MIN_ARMED_DSHOT_VALUE, MAXIMUM_MOTOR_SPEED));
	motor3.send_dshot_value(constrain((int)m3, MIN_ARMED_DSHOT_VALUE, MAXIMUM_MOTOR_SPEED));
	motor4.send_dshot_value(constrain((int)m4, MIN_ARMED_DSHOT_VALUE, MAXIMUM_MOTOR_SPEED));
}