#pragma once

#include <cstdint>
#include <unordered_map>
#include <functional>
#include <chrono>
#include "Status.h"
#include "COMMS/RFD900.h"

// Command IDs matching your radio protocol
enum CommandID : uint8_t {
	FORWARD   = 100,
	BACKWARD  = 101,
	LEFT      = 102,
	RIGHT     = 103,
	PAN_LEFT  = 104,
	PAN_RIGHT = 105,
	GO_UP     = 107,
	GO_DOWN   = 108,
	TOGGLE    = 17,
	P         = 50,
	I         = 51,
	D         = 52,
};

struct ControlInput {
	int16_t pitch		{0};	// forward/backward
	int16_t roll		{0};	// left/right
	int16_t throttle	{0};	// up/down
	int16_t yaw 		{0};	// rotation (optional)
};

// Main control manager
class MovementController {
public:
	MovementController(Status& s, RFD900& rfd900);
	void begin();
	// Call when receiving a command from RFD900
	void executeCommand(CommandID id, uint8_t rawValue);

	// Call periodically in main loop (e.g. every 10–20 ms)
	void update();

	// Get current filtered input state
	ControlInput getInput() const;

	// Returns whether input is valid (not timed out)
	bool isInputActive() const;

	bool isToggled;
	double Kp = 1, Ki = 0, Kd = 1;


private:
	// Timeout in ms after which input is reset
	static constexpr int INPUT_TIMEOUT_MS = 500000;

	std::unordered_map<CommandID, std::function<void(uint8_t)>> commandMap;
	ControlInput targetInput;   // from latest commands
	ControlInput currentInput;  // smoothed input
	std::chrono::steady_clock::time_point lastCommandTime;
	Status& status;
	RFD900& rfd900;

	// Direction handlers
	void handleForward(uint8_t value);
	void handleBackward(uint8_t value);
	void handleLeft(uint8_t value);
	void handleRight(uint8_t value);
	void handleUp(uint8_t value);
	void handleDown(uint8_t value);
	void handlePanLeft(uint8_t value);
	void handlePanRight(uint8_t value);
	void toggle(uint8_t value);
	void P(uint8_t value);
	void I(uint8_t value);
	void D(uint8_t value);

	// Helpers
	void updateCommandMap();
	void applyFailsafeIfTimedOut();
	int16_t mapInput(uint8_t rawValue);
	int16_t smooth(int16_t current, int16_t target);
};

