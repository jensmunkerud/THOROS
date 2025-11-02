#include "MovementController.h"
#include <Arduino.h>

constexpr int16_t INPUT_SCALE = 1000;        // range for input [-1000, +1000]
constexpr uint8_t SPEED_MAX   = 255;
constexpr uint16_t SMOOTHING_ALPHA_NUM = 1;  // numerator
constexpr uint16_t SMOOTHING_ALPHA_DEN = 8;  // denominator => alpha = 0.25

MovementController::MovementController(Status& s, RFD900& rfd900) : status{s}, rfd900{rfd900} {
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
}

void MovementController::executeCommand(CommandID id, uint8_t rawValue) {
	auto it = commandMap.find(id);
	if (it == commandMap.end()) return;

	// float normalized = normalizeInput(rawValue);
	it->second(rawValue);
	lastCommandTime = std::chrono::steady_clock::now();
}

// === Command Handlers ===
void MovementController::handleForward(uint8_t v)  { targetInput.pitch =	targetInput.pitch >= 0 ? v : 0; }
void MovementController::handleBackward(uint8_t v) { targetInput.pitch =	targetInput.pitch <= 0 ? -v : 0; }
void MovementController::handleLeft(uint8_t v)     { targetInput.roll =		targetInput.roll <= 0 ? -v : 0; }
void MovementController::handleRight(uint8_t v)    { targetInput.roll =		targetInput.roll >= 0 ? v : 0; }
void MovementController::handleUp(uint8_t v)       { targetInput.throttle =	targetInput.throttle >= 0 ? v : 0; }
void MovementController::handleDown(uint8_t v)     { targetInput.throttle =	targetInput.throttle <= 0 ? -v : 0;}
void MovementController::handlePanLeft(uint8_t v)  { targetInput.yaw =		targetInput.yaw <= 0 ? -v : 0; }
void MovementController::handlePanRight(uint8_t v) { targetInput.yaw =		targetInput.yaw >= 0 ? v : 0; }

// === Helper Functions ===
int16_t MovementController::mapInput(uint8_t rawValue) {
	return (static_cast<int32_t>(rawValue) * INPUT_SCALE) / 255;
}

int16_t MovementController::smooth(int16_t current, int16_t target) {
	if (current < target) {
		return current + 1;
	} 
	else if (current > target) {
		return current - 1;
	} 
	else {
		return current;
	}
}

// === Update Loop ===
void MovementController::update() {
	uint8_t packet[2];
	while (xQueueReceive(rfd900.commandQueue, packet, 0) == pdTRUE) {
		uint8_t cmd = packet[0];
		uint8_t val = packet[1];
		executeCommand(static_cast<CommandID>(cmd), val);
	}

	// Smooth the current input toward the target
	currentInput.pitch     = smooth(currentInput.pitch,     targetInput.pitch);
	currentInput.roll      = smooth(currentInput.roll,      targetInput.roll);
	currentInput.throttle  = smooth(currentInput.throttle,  targetInput.throttle);
	currentInput.yaw       = smooth(currentInput.yaw,       targetInput.yaw);
	status.speed = currentInput.throttle;
	// applyFailsafeIfTimedOut();
}

void MovementController::applyFailsafeIfTimedOut() {
	auto now = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastCommandTime).count();
	if (elapsed > INPUT_TIMEOUT_MS) {
		targetInput = {};  // Reset all inputs
	}
}

ControlInput MovementController::getInput() const {
	return currentInput;
}

bool MovementController::isInputActive() const {
	auto now = std::chrono::steady_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastCommandTime).count();
	return elapsed < INPUT_TIMEOUT_MS;
}
