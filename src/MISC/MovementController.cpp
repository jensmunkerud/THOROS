#include "MovementController.h"
#include "Motor.h"
#include "Logger.h"
#include <Arduino.h>
#include <cmath>

extern Motor motor;
extern Logger logger;

constexpr float MODE_LATERAL_EPSILON {0.15f};
constexpr float MODE_YAW_EPSILON {0.15f};
constexpr float MODE_LANDED_THROTTLE {600.0f};


MovementController::MovementController(Telemetry& tel, Drone& drone, RFD900& rfd900) : 
	telemetry{tel},
	drone{drone},
	rfd900{rfd900},
	isToggled{false},
	canApplyFailSafe{true},
	canChangeSpeed{true},
	movementSpeed{400}
{
	generateCommandMap();
}

void MovementController::begin() {
	lastCommandTime = std::chrono::steady_clock::now();
}

void MovementController::updateRunningCommands() {
	// Set all command values to their new values
	for (const auto& entry : newCommands) {
		executeCommand(entry.first, entry.second);
	}

	// Detect commands that were active but are now gone
	for (const auto& oldCmd : runningCommands) {
		if (newCommands.find(oldCmd) == newCommands.end()) {
			executeCommand(oldCmd, 0);
		}
	}

	// Update activeCommands for next frame
	runningCommands.clear();
	for (const auto& entry : newCommands) {
		runningCommands.insert(entry.first);
	}
}

void MovementController::executeCommand(CommandID id, uint8_t rawValue) {
	auto it = commandMap.find(id);
	if (it == commandMap.end()) {return;}
	it->second(rawValue);
}

void MovementController::generateCommandMap() {
	commandMap[CommandID::FORWARD]			= [this](uint8_t v){ handleForward(v); };
	commandMap[CommandID::BACKWARD]			= [this](uint8_t v){ handleBackward(v); };
	commandMap[CommandID::LEFT]				= [this](uint8_t v){ handleLeft(v); };
	commandMap[CommandID::RIGHT]			= [this](uint8_t v){ handleRight(v); };
	commandMap[CommandID::PAN_LEFT]			= [this](uint8_t v){ handlePanLeft(v); };
	commandMap[CommandID::PAN_RIGHT]		= [this](uint8_t v){ handlePanRight(v); };
	commandMap[CommandID::GO_UP]			= [this](uint8_t v){ handleUp(v); };
	commandMap[CommandID::GO_DOWN]			= [this](uint8_t v){ handleDown(v); };
	commandMap[CommandID::TOGGLE]			= [this](uint8_t v){ toggle(v); };
	commandMap[CommandID::SPEED_UP]			= [this](uint8_t v){ increaseSpeed(v); };
	commandMap[CommandID::SPEED_DOWN]		= [this](uint8_t v){ decreaseSpeed(v); };
	commandMap[CommandID::LOG_TOGGLE]		= [this](uint8_t v){ log_toggle(v); };
	commandMap[CommandID::KILL]				= [this](uint8_t v){ clearInputs(true); canApplyFailSafe = false; motor.Kill();};
	commandMap[CommandID::ARM]				= [this](uint8_t v){ handleArm(v); };
}


// === Command Handlers ===
void MovementController::handleForward(uint8_t v)  { target.pitch = -static_cast<float>(v); }
void MovementController::handleBackward(uint8_t v) { target.pitch = static_cast<float>(v); }
void MovementController::handleLeft(uint8_t v)     { target.roll = static_cast<float>(v); }
void MovementController::handleRight(uint8_t v)    { target.roll = -static_cast<float>(v); }
void MovementController::handlePanLeft(uint8_t v)  { target.yaw = v > 0 ? v/255.0f : 0.0f; }
void MovementController::handlePanRight(uint8_t v) { target.yaw = v > 0 ? -v/255.0f : 0.0f; }
void MovementController::handleUp(uint8_t v)       { target.throttle = v > 0 ? v/255.0f : 0.0f; }
void MovementController::handleDown(uint8_t v)     { target.throttle = v > 0 ? -v/255.0f : 0.0f; }
void MovementController::toggle(uint8_t v) {isToggled = not isToggled; return; Serial.print("Toggled , it is now: "); Serial.println(isToggled);}
void MovementController::increaseSpeed(uint8_t v)  {if (canChangeSpeed && v > 0) {movementSpeed = constrain(movementSpeed + v, 50, 500); canChangeSpeed = false;} Serial.print("Sped up, speed: "); Serial.println(movementSpeed); if (v == 0) {canChangeSpeed = true;}}
void MovementController::decreaseSpeed(uint8_t v)  {if (canChangeSpeed && v > 0) {movementSpeed = constrain(movementSpeed - v, 50, 500); canChangeSpeed = false;} if (v == 0) {canChangeSpeed = true;}}
void MovementController::log_toggle(uint8_t v)     {
	if (v == 0) {
		return;
	}

	if (runningCommands.find(CommandID::LOG_TOGGLE) != runningCommands.end()) {
		return;
	}

	if (logger.isLogging()) {
		logger.stopLog();
	} else {
		logger.startLog();
	}
}

void MovementController::handleArm(uint8_t v) {
	if (v == 0) {
		armButtonActive = false;
		return;
	}

	auto now = std::chrono::steady_clock::now();
	if (!armButtonActive) {
		armButtonActive = true;
		armPressedAt = now;
		return;
	}

	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - armPressedAt).count();
	if (elapsed >= ARM_HOLD_MS) {
		{
			DroneLockGuard droneLock(drone);
			if (drone.mode == FlightMode::DISARMED) {
				drone.flightControls = {};
				drone.mode = FlightMode::ARMED;
			}
		}
		clearInputs(true);
		armButtonActive = false; // prevent repeated re-arming
	}
}


float MovementController::smooth(float current, float target, float sensitivity, float deltaMs)
{
	if (current == target)
		return current;

	float step = sensitivity * (deltaMs / 1000.0f);

	// Prevents overshoot
	if (current < target) {
		current += step;
		if (current > target) current = target;
	} else {
		current -= step;
		if (current < target) current = target;
	}

	return current;
}


// === Update Loop ===
void MovementController::update() {
	deltaMs = millis() - lastTime;
	lastTime = millis();

	if (xQueueReceive(rfd900.getCommandQueue(), &received, 0) == pdTRUE) {
		newCommands.clear();
		// Build map of all incoming commands
		for (int i = 0; i < received.numCmds; i++) {
			CommandID id = static_cast<CommandID>(received.commands[i].command);
			uint8_t val = received.commands[i].value;
			newCommands[id] = val;
		}
		updateRunningCommands();
		if (received.numCmds > 0) {
			canApplyFailSafe = true;
			lastCommandTime = std::chrono::steady_clock::now();
		} else {
			// PUT DRONE EITHER IN LANDED, HOVERING OR ARMED MODE!!!
		}
	}

	FlightControls controls;
	{
		DroneLockGuard droneLock(drone);
		controls = drone.flightControls;
	}

	// Apply output signals from MovementController to a local controls snapshot.
	controls.pitch = constrain(
		smooth(controls.pitch, target.pitch, TILT_SPEED, deltaMs),
		-MAX_TILT_ANGLE, MAX_TILT_ANGLE
	);
	controls.roll = constrain(
		smooth(controls.roll, target.roll, TILT_SPEED, deltaMs),
		-MAX_TILT_ANGLE, MAX_TILT_ANGLE
	);

	if (target.yaw != 0.0f) {
		controls.yaw = constrain(smooth(controls.yaw, controls.yaw + PAN_SPEED * target.yaw, PAN_SPEED, deltaMs), -MAX_PAN, MAX_PAN);
	}
	
	if (target.throttle != 0.0f) {
		controls.throttle = constrain(smooth(controls.throttle, controls.throttle + THROTTLE_SPEED * target.throttle, THROTTLE_SPEED, deltaMs), 0.0f, MAX_THROTTLE);
	}

	{
		DroneLockGuard droneLock(drone);
		drone.flightControls = controls;
	}

	updateFlightMode(controls);

	applyFailsafeIfTimedOut();
}

bool MovementController::hasLateralInput(const FlightControls& controls) const {
	return fabsf(controls.pitch) > MODE_LATERAL_EPSILON || fabsf(controls.roll) > MODE_LATERAL_EPSILON;
}

bool MovementController::hasYawInput(const FlightControls& controls) const {
	return fabsf(controls.yaw) > MODE_YAW_EPSILON;
}

void MovementController::updateFlightMode(const FlightControls& controls) {
	FlightMode currentMode;
	{
		DroneLockGuard droneLock(drone);
		currentMode = drone.mode;
	}

	if (currentMode == FlightMode::DISARMED) {
		return;
	}

	const bool lateralActive = hasLateralInput(controls);
	const bool yawActive = hasYawInput(controls);
	const bool movementActive = lateralActive || yawActive;
	const float throttle = controls.throttle;

	FlightMode nextMode = FlightMode::ARMED;
	if (throttle <= MODE_LANDED_THROTTLE && !movementActive) {
		nextMode = FlightMode::LANDED;
	} else if (throttle > MODE_LANDED_THROTTLE && !movementActive) {
		nextMode = FlightMode::HOVERING;
	} else if (movementActive || throttle > MODE_LANDED_THROTTLE) {
		nextMode = FlightMode::MOVING;
	}

	if (nextMode != currentMode) {
		DroneLockGuard droneLock(drone);
		drone.mode = nextMode;
	}
}

// Disable user input if we loose radio communication while moving.
void MovementController::applyFailsafeIfTimedOut() {
	auto now = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastCommandTime).count();
	if (elapsed > MOVEMENT_TIMEOUT_MS) {
		clearInputs();
	}
}

void MovementController::clearInputs(bool clearThrottle) {
	if (canApplyFailSafe || clearThrottle) {
		target.pitch = 0;
		target.roll = 0;
		target.yaw = 0;
		target.throttle = 0;
	}
	canApplyFailSafe = false;
}