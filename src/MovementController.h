#pragma once
#include <cstdint>
#include <unordered_map>
#include <functional>
#include <chrono>
#include "Status.h"
#include "COMMS/RFD900.h"
#include "set"

constexpr int SENSITIVITY {200};
static constexpr int MOVEMENT_TIMEOUT_MS {55}; // 2x 80ms which is the sending interval


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
	Pd        = 51,
	I         = 52,
	Id        = 53,
	D         = 54,
	Dd        = 55,
	KILL      = 254
};


struct ControlInput {
	float pitch		{0};	// forward/backward
	float roll		{0};	// left/right
	float throttle	{0};	// up/down
	float yaw 		{0};	// rotation (optional)
};

// Main control manager
class MovementController {
public:
	MovementController(Status& s, RFD900& rfd900);
	void begin();
	void executeCommand(CommandID id, uint8_t rawValue);
	void update();
	ControlInput getInput() const;
	bool isInputActive() const;
	bool isToggled;
	double Kp = 1, Ki = 0, Kd = 1;
	std::array<bool, 8> commands_in_action;
	ControlInput currentInput;  // smoothed input
	void clearInputs(bool clearThrottle = false);


private:

	std::unordered_map<CommandID, std::function<void(uint8_t)>> commandMap;
	ControlInput targetInput;   // from latest commands
	std::chrono::steady_clock::time_point lastCommandTime;
	Status& status;
	RFD900& rfd900;
	RFDCommandPacket received;
	std::unordered_map<CommandID, uint8_t> newCommands;
	std::set<CommandID> oldCommands;
	std::chrono::steady_clock::time_point lastAutoProcessTime;

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
	void Pd(uint8_t value);
	void I(uint8_t value);
	void Id(uint8_t value);
	void D(uint8_t value);
	void Dd(uint8_t value);

	int verticalSpeed {0};
	int count {0};
	long lastTime{0};
	long deltaTime{0};
	void deleteCommand(uint8_t command_id);
	void updateRunningCommands();
	void controlTimeouts();

	// Helpers
	void updateCommandMap();
	void applyFailsafeIfTimedOut();
	bool canApplyFailSafe;
	int16_t mapInput(uint8_t rawValue);
	float smooth(float current, float target, float sensitivity, float deltaTime);
};

