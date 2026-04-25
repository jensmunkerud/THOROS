#include "MovementController.h"
#include "Motor.h"
#include <Arduino.h>
#include <cmath>

extern Motor motor;

constexpr float MODE_LATERAL_EPSILON {0.15f};
constexpr float MODE_YAW_EPSILON {0.15f};
constexpr float MODE_LANDED_THROTTLE {15.0f};
constexpr float MODE_HOVERING_THROTTLE {80.0f};


MovementController::MovementController(Telemetry& tel, Drone& drone, RFD900& rfd900) : 
telemetry{tel},
drone{drone},
rfd900{rfd900},
isToggled{false},
canApplyFailSafe{true},
canChangeSpeed(true),
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
	for (const auto& oldCmd : oldCommands) {
		if (newCommands.find(oldCmd) == newCommands.end()) {
			executeCommand(oldCmd, 0);
		}
	}

	// Update activeCommands for next frame
	oldCommands.clear();
	for (const auto& entry : newCommands) {
		oldCommands.insert(entry.first);
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
	commandMap[CommandID::P]				= [this](uint8_t v){ P(v); };
	commandMap[CommandID::Pd]				= [this](uint8_t v){ Pd(v); };
	commandMap[CommandID::I]				= [this](uint8_t v){ I(v); };
	commandMap[CommandID::Id]				= [this](uint8_t v){ Id(v); };
	commandMap[CommandID::D]				= [this](uint8_t v){ D(v); };
	commandMap[CommandID::Dd]				= [this](uint8_t v){ Dd(v); };
	commandMap[CommandID::KILL]				= [this](uint8_t v){ clearInputs(true); canApplyFailSafe = false; motor.Kill();};
	commandMap[CommandID::SPEED_UP]			= [this](uint8_t v){ increaseSpeed(v); };
	commandMap[CommandID::SPEED_DOWN]		= [this](uint8_t v){ decreaseSpeed(v); };
}


// === Command Handlers ===
void MovementController::handleForward(uint8_t v)  { target.pitch = static_cast<float>(v); }
void MovementController::handleBackward(uint8_t v) { target.pitch = -static_cast<float>(v); }
void MovementController::handleLeft(uint8_t v)     { target.roll = -static_cast<float>(v); }
void MovementController::handleRight(uint8_t v)    { target.roll = static_cast<float>(v); }
void MovementController::handlePanLeft(uint8_t v)  { target.yaw = v > 0 ? v/255 : 0.0f; }
void MovementController::handlePanRight(uint8_t v) { target.yaw = v > 0 ? -v/255 : 0.0f; }
void MovementController::handleUp(uint8_t v)       { target.throttle = v > 0 ? v/255 : 0.0f; }
void MovementController::handleDown(uint8_t v)     { target.throttle = v > 0 ? -v/255 : 0.0f; }
void MovementController::toggle(uint8_t v) {isToggled = not isToggled; return; Serial.print("Toggled , it is now: "); Serial.println(isToggled);}
void MovementController::increaseSpeed(uint8_t v) {if (canChangeSpeed && v > 0) {movementSpeed = constrain(movementSpeed + v, 50, 500); canChangeSpeed = false;} Serial.print("Sped up, speed: "); Serial.println(movementSpeed); if (v == 0) {canChangeSpeed = true;}}
void MovementController::decreaseSpeed(uint8_t v) {if (canChangeSpeed && v > 0) {movementSpeed = constrain(movementSpeed - v, 50, 500); canChangeSpeed = false;} if (v == 0) {canChangeSpeed = true;}}
void MovementController::P(uint8_t v) {Kp = max(0, (int)Kp + v);}
void MovementController::Pd(uint8_t v) {Kp = max(0, (int)Kp - v);}
void MovementController::I(uint8_t v) {Ki = max(0, (int)Ki + v);}
void MovementController::Id(uint8_t v) {Ki = max(0, (int)Ki - v);}
void MovementController::D(uint8_t v) {Kd = max(0, (int)Kd + v);}
void MovementController::Dd(uint8_t v) {Kd = max(0, (int)Kd - v);}


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
		DroneLockGuard lock(drone);
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
		controls.yaw += PAN_SPEED * target.yaw * deltaMs;
	} else {
		controls.yaw = smooth(controls.yaw, 0.0f, PAN_SPEED, deltaMs);
	}
	if (target.throttle != 0.0f) {
		controls.throttle += THROTTLE_SPEED * target.throttle * deltaMs;
	} else {
		controls.throttle = smooth(controls.throttle, 0.0f, THROTTLE_SPEED, deltaMs);
	}

	controls.yaw = constrain(controls.yaw, -180.0f, 180.0f);
	controls.throttle = constrain(controls.throttle, 0.0f, 1500.0f);

	{
		DroneLockGuard lock(drone);
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
		DroneLockGuard lock(drone);
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
	} else if (throttle >= MODE_HOVERING_THROTTLE && !movementActive) {
		nextMode = FlightMode::HOVERING;
	} else if (movementActive || throttle > MODE_LANDED_THROTTLE) {
		nextMode = FlightMode::MOVING;
	}

	if (nextMode != currentMode) {
		DroneLockGuard lock(drone);
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