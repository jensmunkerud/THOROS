#include "MovementController.h"
#include <Arduino.h>
#include <cmath>


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
	commandMap[CommandID::KILL]				= [this](uint8_t v){ clearInputs(true); canApplyFailSafe = false; };
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
		}
	}

	// Apply output signals from MovementController to drone flightControls state.
	// Pitch & roll move and reach target
	drone.flightControls.pitch = constrain(
		smooth(drone.flightControls.pitch, target.pitch, TILT_SPEED, deltaMs),
		-MAX_TILT_ANGLE, MAX_TILT_ANGLE
	);
	drone.flightControls.roll = constrain(
		smooth(drone.flightControls.roll, target.roll, TILT_SPEED, deltaMs),
		-MAX_TILT_ANGLE, MAX_TILT_ANGLE
	);

	// Yaw and throttle continually add up
	if (target.yaw != 0.0f) {
		drone.flightControls.yaw += PAN_SPEED * target.yaw * deltaMs;
	} else {
		drone.flightControls.yaw = smooth(drone.flightControls.yaw, 0.0f, PAN_SPEED, deltaMs);
	}
	if (target.throttle != 0.0f) {
		drone.flightControls.throttle += THROTTLE_SPEED * target.throttle * deltaMs;
	} else {
		drone.flightControls.throttle = smooth(drone.flightControls.throttle, 0.0f, THROTTLE_SPEED, deltaMs);
	}

	drone.flightControls.yaw = constrain(drone.flightControls.yaw, -180.0f, 180.0f);
	drone.flightControls.throttle = constrain(drone.flightControls.throttle, 0.0f, 1500.0f);

	applyFailsafeIfTimedOut();
}

void MovementController::applyFailsafeIfTimedOut() {
	auto now = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastCommandTime).count();
	if (elapsed > MOVEMENT_TIMEOUT_MS) {
		clearInputs(false);
	}
}

void MovementController::clearInputs(bool clearThrottle) {
	if (canApplyFailSafe) {
		target.pitch = 0;
		target.roll = 0;
		target.yaw = 0;
		target.throttle = 0;
	}
	canApplyFailSafe = false;
}