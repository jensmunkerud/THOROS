#pragma once

#include <cstdint>
#include <unordered_map>
#include <functional>
#include <chrono>

// Command IDs matching your radio protocol
enum CommandID : uint8_t {
	FORWARD   = 100,
	BACKWARD  = 101,
	LEFT      = 102,
	RIGHT     = 103,
	PAN_LEFT  = 104,
	PAN_RIGHT = 105,
	GO_UP     = 107,
	GO_DOWN   = 108
};

// Represents normalized flight input [-1.0, 1.0]
struct ControlInput {
	float pitch = 0.0f;     // forward/backward
	float roll = 0.0f;      // left/right
	float throttle = 0.0f;  // up/down
	float yaw = 0.0f;       // rotation (optional)
};

// Main control manager
class MovementController {
public:
	MovementController();

	// Call when receiving a command from RFD900
	void executeCommand(CommandID id, int16_t rawValue);

	// Call periodically in main loop (e.g. every 10–20 ms)
	void update();

	// Get current filtered input state
	ControlInput getInput() const;

	// Returns whether input is valid (not timed out)
	bool isInputActive() const;

private:
	std::unordered_map<CommandID, std::function<void(float)>> commandMap;
	ControlInput targetInput;   // from latest commands
	ControlInput currentInput;  // smoothed input
	std::chrono::steady_clock::time_point lastCommandTime;

	// Timeout in ms after which input is reset
	static constexpr int INPUT_TIMEOUT_MS = 500;
	static constexpr float SMOOTHING_ALPHA = 0.2f; // 0.0–1.0

	// Direction handlers
	void handleForward(float value);
	void handleBackward(float value);
	void handleLeft(float value);
	void handleRight(float value);
	void handleUp(float value);
	void handleDown(float value);
	void handlePanLeft(float value);
	void handlePanRight(float value);

	// Helpers
	void updateCommandMap();
	void applyFailsafeIfTimedOut();
	float normalizeInput(int16_t rawValue);
	float smooth(float current, float target, float alpha);
};

