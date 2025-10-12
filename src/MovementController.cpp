#include "MovementController.h"
#include <Arduino.h>

MovementController::MovementController() {
	updateCommandMap();
	lastCommandTime = std::chrono::steady_clock::now();
}

void MovementController::updateCommandMap() {
	commandMap[CommandID::FORWARD]   = [this](float v){ handleForward(v); };
	commandMap[CommandID::BACKWARD]  = [this](float v){ handleBackward(v); };
	commandMap[CommandID::LEFT]      = [this](float v){ handleLeft(v); };
	commandMap[CommandID::RIGHT]     = [this](float v){ handleRight(v); };
	commandMap[CommandID::PAN_LEFT]  = [this](float v){ handlePanLeft(v); };
	commandMap[CommandID::PAN_RIGHT] = [this](float v){ handlePanRight(v); };
	commandMap[CommandID::GO_UP]     = [this](float v){ handleUp(v); };
	commandMap[CommandID::GO_DOWN]   = [this](float v){ handleDown(v); };
}

void MovementController::executeCommand(CommandID id, int16_t rawValue) {
	auto it = commandMap.find(id);
	if (it == commandMap.end()) return;

	float normalized = normalizeInput(rawValue);
	it->second(normalized);
	lastCommandTime = std::chrono::steady_clock::now();
}

// === Command Handlers ===
void MovementController::handleForward(float v)  { targetInput.pitch =  v; }
void MovementController::handleBackward(float v) { targetInput.pitch = -v; }
void MovementController::handleLeft(float v)     { targetInput.roll  = -v; }
void MovementController::handleRight(float v)    { targetInput.roll  =  v; }
void MovementController::handleUp(float v)       { targetInput.throttle =  v; }
void MovementController::handleDown(float v)     { targetInput.throttle = -v; }
void MovementController::handlePanLeft(float v)  { targetInput.yaw = -v; }
void MovementController::handlePanRight(float v) { targetInput.yaw =  v; }

// === Helper Functions ===
float MovementController::normalizeInput(int16_t rawValue) {
	// Map [-32768, 32767] to [-1.0, 1.0]
	return constrain(static_cast<float>(rawValue) / 32767.0f, -1.0f, 1.0f);
}

float MovementController::smooth(float current, float target, float alpha) {
	return current + alpha * (target - current);
}

// === Update Loop ===
void MovementController::update() {
	// Smooth the current input toward the target
	currentInput.pitch     = smooth(currentInput.pitch,     targetInput.pitch,     SMOOTHING_ALPHA);
	currentInput.roll      = smooth(currentInput.roll,      targetInput.roll,      SMOOTHING_ALPHA);
	currentInput.throttle  = smooth(currentInput.throttle,  targetInput.throttle,  SMOOTHING_ALPHA);
	currentInput.yaw       = smooth(currentInput.yaw,       targetInput.yaw,       SMOOTHING_ALPHA);

	applyFailsafeIfTimedOut();
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
