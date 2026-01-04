#include "MovementController.h"
#include <Arduino.h>

constexpr int16_t INPUT_SCALE = 1000;        // range for input [-1000, +1000]
constexpr uint8_t SPEED_MAX   = 255;
constexpr uint16_t SMOOTHING_ALPHA_NUM = 1;  // numerator
constexpr uint16_t SMOOTHING_ALPHA_DEN = 8;  // denominator => alpha = 0.25

MovementController::MovementController(Status& s, RFD900& rfd900) : status{s}, rfd900{rfd900} , isToggled{false}, last_command_time_map{}, commands_in_action{}{
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
void MovementController::handleForward(uint8_t v)  { targetInput.pitch =	targetInput.pitch >= 0 ? v : 0;		last_command_time_map[0] = millis();}
void MovementController::handleBackward(uint8_t v) { targetInput.pitch =	targetInput.pitch <= 0 ? -v : 0;	last_command_time_map[1] = millis();}
void MovementController::handleLeft(uint8_t v)     { targetInput.roll =		targetInput.roll <= 0 ? -v : 0;		last_command_time_map[2] = millis();}
void MovementController::handleRight(uint8_t v)    { targetInput.roll =		targetInput.roll >= 0 ? v : 0;		last_command_time_map[3] = millis();}
void MovementController::handlePanLeft(uint8_t v)  { targetInput.yaw =		targetInput.yaw <= 0 ? -v : 0;		last_command_time_map[4] = millis();}
void MovementController::handlePanRight(uint8_t v) { targetInput.yaw =		targetInput.yaw >= 0 ? v : 0;		last_command_time_map[5] = millis();}
void MovementController::handleUp(uint8_t v)       { targetInput.throttle =	v;									last_command_time_map[6] = millis();}
void MovementController::handleDown(uint8_t v)     { verticalSpeed =		0;									last_command_time_map[7] = millis();}
void MovementController::toggle(uint8_t v) {isToggled = not isToggled; Serial.print("Toggled , it is now: "); Serial.println(isToggled);}
void MovementController::P(uint8_t v) {Kp = max(0, (int)Kp + v);}
void MovementController::Pd(uint8_t v) {Kp = max(0, (int)Kp - v);}
void MovementController::I(uint8_t v) {Ki = max(0, (int)Ki + v);}
void MovementController::Id(uint8_t v) {Ki = max(0, (int)Ki - v);}
void MovementController::D(uint8_t v) {Kd = max(0, (int)Kd + v);}
void MovementController::Dd(uint8_t v) {Kd = max(0, (int)Kd - v);}

// === Helper Functions ===
int16_t MovementController::mapInput(uint8_t rawValue) {
	return (static_cast<int32_t>(rawValue) * INPUT_SCALE) / 255;
}

long MovementController::smooth(long current, long target, int16_t sensitivity, long deltaTime, bool returnToZero) {
	if (current < target) {
		return current + sensitivity * deltaTime / 1000;
	} 
	else if (current > target) {
		return current - sensitivity * deltaTime / 1000;
	} 
	else {
		return current;
	}
}

// void MovementController::pingCommand(uint8_t command_id, bool shouldDisable) {
// 	switch (command_id) {
// 		case 6: // 107 up
// 			if (targetInput.throttle > 0) {
// 				if (shouldDisable) {targetInput.throttle = 0; break;}
// 				break;
// 			}
// 		case 108:
// 			if (targetInput.throttle < 0) {
// 				targetInput.throttle = 0;
// 			}
// 	}
// }

// void MovementController::controlTimeouts() {
// 	for (int i = 0; i < sizeof(last_command_time_map); i++) {
// 		if (last_command_time_map[i] > MOVEMENT_TIMEOUT_MS) {
// 		}
// 	}
// }

// === Update Loop ===
void MovementController::update() {
	deltaTime = millis() - lastTime;
	lastTime = millis();

	uint8_t packet[2];
	if (xQueueReceive(rfd900.getCommandQueue(), packet, 0) == pdTRUE) {
		executeCommand(static_cast<CommandID>(packet[0]), packet[1]);
		// Serial.print("Executing command: ");
		// Serial.println(packet[0]);
	}
	// if (verticalSpeed == 0) {targetInput.throttle = 0;} else {
	// 	targetInput.throttle += verticalSpeed * deltaTime / 100;
	// }
	// Smooth the current input toward the target
	currentInput.pitch     = smooth(currentInput.pitch,     targetInput.pitch,		1000, deltaTime, false);
	currentInput.roll      = smooth(currentInput.roll,      targetInput.roll,		1000, deltaTime, false);
	currentInput.throttle  = smooth(currentInput.throttle,  targetInput.throttle,	1000, deltaTime, true);
	currentInput.yaw       = smooth(currentInput.yaw,       targetInput.yaw,		1000, deltaTime, false);
	status.speed = currentInput.throttle;
	Serial.println(currentInput.throttle);
	applyFailsafeIfTimedOut();
}

void MovementController::applyFailsafeIfTimedOut() {
	auto now = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastCommandTime).count();
	if (elapsed > MOVEMENT_TIMEOUT_MS) {
		targetInput = {};  // Reset all inputs
		Serial.println("TIMED OUT INPUT");
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
