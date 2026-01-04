#include "MovementController.h"
#include <Arduino.h>

MovementController::MovementController(Status& s, RFD900& rfd900) : status{s}, rfd900{rfd900} , isToggled{false}, last_command_time_map{0, 0, 0, 0, 0, 0, 0, 0}, commands_in_action{0, 0, 0, 0, 0, 0, 0, 0}{
	updateCommandMap();
}

void MovementController::begin() {
	lastCommandTime = std::chrono::steady_clock::now();
}

void MovementController::updateCommandMap() {
	commandMap[CommandID::FORWARD]   = [this](uint8_t v){ handleForward(v); };
	commandMap[CommandID::BACKWARD]  = [this](uint8_t v){ handleBackward(v); };
	commandMap[CommandID::LEFT]      = [this](uint8_t v){ handleLeft(v); };
	commandMap[CommandID::RIGHT]     = [this](uint8_t v){ handleRight(v); };
	commandMap[CommandID::PAN_LEFT]  = [this](uint8_t v){ handlePanLeft(v); };
	commandMap[CommandID::PAN_RIGHT] = [this](uint8_t v){ handlePanRight(v); };
	commandMap[CommandID::GO_UP]     = [this](uint8_t v){ handleUp(v); };
	commandMap[CommandID::GO_DOWN]   = [this](uint8_t v){ handleDown(v); };
	commandMap[CommandID::TOGGLE]    = [this](uint8_t v){ toggle(v); };
	commandMap[CommandID::P]         = [this](uint8_t v){ P(v); };
	commandMap[CommandID::Pd]        = [this](uint8_t v){ Pd(v); };
	commandMap[CommandID::I]         = [this](uint8_t v){ I(v); };
	commandMap[CommandID::Id]        = [this](uint8_t v){ Id(v); };
	commandMap[CommandID::D]         = [this](uint8_t v){ D(v); };
	commandMap[CommandID::Dd]        = [this](uint8_t v){ Dd(v); };
}

void MovementController::executeCommand(CommandID id, uint8_t rawValue) {
	auto it = commandMap.find(id);
	if (it == commandMap.end()) return;

	// float normalized = normalizeInput(rawValue);
	it->second(rawValue);
	lastCommandTime = std::chrono::steady_clock::now();
}

// === Command Handlers ===
void MovementController::handleForward(uint8_t v)  { targetInput.pitch =	targetInput.pitch > 0 ? v : 0;		last_command_time_map[0] = millis(); commands_in_action[0] = true;}
void MovementController::handleBackward(uint8_t v) { targetInput.pitch =	targetInput.pitch < 0 ? -v : 0;		last_command_time_map[1] = millis(); commands_in_action[1] = true;}
void MovementController::handleLeft(uint8_t v)     { targetInput.roll =		targetInput.roll < 0 ? -v : 0;		last_command_time_map[2] = millis(); commands_in_action[2] = true;}
void MovementController::handleRight(uint8_t v)    { targetInput.roll =		targetInput.roll > 0 ? v : 0;		last_command_time_map[3] = millis(); commands_in_action[3] = true;}
void MovementController::handlePanLeft(uint8_t v)  { targetInput.yaw =		targetInput.yaw < 0 ? -v : 0;		last_command_time_map[4] = millis(); commands_in_action[4] = true;}
void MovementController::handlePanRight(uint8_t v) { targetInput.yaw =		targetInput.yaw > 0 ? v : 0;		last_command_time_map[5] = millis(); commands_in_action[5] = true;}
void MovementController::handleUp(uint8_t v)       { targetInput.throttle =	v > 0 ? v*16 : currentInput.throttle;	last_command_time_map[6] = millis(); commands_in_action[6] = true;}
void MovementController::handleDown(uint8_t v)     { targetInput.throttle = v > 0 ? -v*16 : currentInput.throttle;	last_command_time_map[7] = millis(); commands_in_action[7] = true;}
void MovementController::toggle(uint8_t v) {isToggled = not isToggled; Serial.print("Toggled , it is now: "); Serial.println(isToggled);}
void MovementController::P(uint8_t v) {Kp = max(0, (int)Kp + v);}
void MovementController::Pd(uint8_t v) {Kp = max(0, (int)Kp - v);}
void MovementController::I(uint8_t v) {Ki = max(0, (int)Ki + v);}
void MovementController::Id(uint8_t v) {Ki = max(0, (int)Ki - v);}
void MovementController::D(uint8_t v) {Kd = max(0, (int)Kd + v);}
void MovementController::Dd(uint8_t v) {Kd = max(0, (int)Kd - v);}


float MovementController::smooth(float current, float target, float sensitivity, float deltaTime)
{
	if (current == target)
		return current;

	float step = sensitivity * (deltaTime / 1000.0f);

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


void MovementController::deleteCommand(uint8_t command_id) {
	switch (command_id) {
		case 0: // 100 forward
			if (targetInput.pitch < 0) {
				targetInput.pitch = 0;
				break;
			}
		case 1: // 101 backward
			if (targetInput.pitch > 0) {
				targetInput.pitch = 0;
				break;
			}
		case 2: // 102 left
			if (targetInput.roll < 0) {
				targetInput.roll = 0;
				break;
			}
		case 3: // 103 right
			if (targetInput.roll > 0) {
				targetInput.roll = 0;
				break;
			}
		case 4: // 104 pan left
			if (targetInput.yaw < 0) {
				targetInput.yaw = 0;
				break;
			}
		case 5: // 105 pan right
			if (targetInput.yaw > 0) {
				targetInput.yaw = 0;
				break;
			}
		case 6: // 107 up
			break;
			if (targetInput.throttle > 0) {
				targetInput.throttle = 0;
				break;
			}
		case 7: // 108 down
			break;
			if (targetInput.throttle < 0) {
				targetInput.throttle = 0;
				break;
			}
	}
}

void MovementController::controlTimeouts() {
	for (int i = 0; i < sizeof(commands_in_action); i++) {
		if (!commands_in_action[i]) {continue;}
		if (millis() - last_command_time_map[i] > MOVEMENT_TIMEOUT_MS) {
			commands_in_action[i] = false;
			deleteCommand(i);
			Serial.print("TIMED OUT ACTION ");
			Serial.print( i);
			Serial.print(" with delay: ");
			Serial.println(millis() - last_command_time_map[i]);
		}
	}
}

// === Update Loop ===
void MovementController::update() {
	deltaTime = millis() - lastTime;
	lastTime = millis();
	controlTimeouts();

	uint8_t packet[2];
	if (xQueueReceive(rfd900.getCommandQueue(), packet, 0) == pdTRUE) {
		executeCommand(static_cast<CommandID>(packet[0]), packet[1]);
		// Serial.print("Executing command: ");
		// Serial.println(packet[0]);
	}
	// Smooth the current input toward the target
	currentInput.pitch     = constrain(smooth(currentInput.pitch,     targetInput.pitch,		SENSITIVITY, deltaTime), -1000, 1000);
	currentInput.roll      = constrain(smooth(currentInput.roll,      targetInput.roll,		SENSITIVITY, deltaTime), -1000, 1000);
	currentInput.throttle  = constrain(smooth(currentInput.throttle,  targetInput.throttle,	SENSITIVITY, deltaTime), 0, 2000);
	currentInput.yaw       = constrain(smooth(currentInput.yaw,       targetInput.yaw,		SENSITIVITY, deltaTime), -1000, 1000);
	status.speed = static_cast<int>(currentInput.throttle);
	Serial.print("Target: ");
	Serial.print(targetInput.throttle);
	Serial.print(" Current: ");
	Serial.println(currentInput.throttle);
	// applyFailsafeIfTimedOut();
}

void MovementController::applyFailsafeIfTimedOut() {
	auto now = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastCommandTime).count();
	if (elapsed > MOVEMENT_TIMEOUT_MS) {
		targetInput = {};  // Reset all inputs
	}
}

ControlInput MovementController::getInput() const {
	return currentInput;
}

bool MovementController::isInputActive() const {
	auto now = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastCommandTime).count();
	return elapsed < MOVEMENT_TIMEOUT_MS;
}
