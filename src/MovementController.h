#pragma once
#include <cstdint>
#include <unordered_map>
#include <functional>
#include <chrono>
#include "Datatypes.h"
#include "COMMS/RFD900.h"
#include "set"

constexpr int SENSITIVITY {50};
constexpr float MAX_TILT_ANGLE {30.0f};
constexpr float PAN_SPEED {10.0f};
constexpr float THROTTLE_SPEED {10.0f};
static constexpr int MOVEMENT_TIMEOUT_MS {55}; // 2x 80ms which is the sending interval


// Command IDs matching your radio protocol
enum CommandID : uint8_t {
	FORWARD		= 100,
	BACKWARD	= 101,
	LEFT		= 102,
	RIGHT		= 103,
	PAN_LEFT	= 104,
	PAN_RIGHT	= 105,
	GO_UP		= 107,
	GO_DOWN 	= 108,
	TOGGLE		= 17,
	P			= 50,
	Pd			= 51,
	I			= 52,
	Id			= 53,
	D			= 54,
	Dd			= 55,
	SPEED_UP	= 109,
	SPEED_DOWN	= 110,
	KILL		= 254,
};
// Main control manager
class MovementController {
public:
	MovementController(Telemetry& tel, DroneState& droneState, RFD900& rfd900);
	void begin();
	void update();
	ControlInput getInput() const;
	bool isToggled;
	double Kp = 1, Ki = 0, Kd = 1;
	void clearInputs(bool clearThrottle = false);

private:
	
	ControlInput currentInput;
	ControlInput targetInput;
	Telemetry& telemetry;
	DroneState& droneState;
	RFD900& rfd900;
	std::unordered_map<CommandID, std::function<void(uint8_t)>> commandMap;
	std::chrono::steady_clock::time_point lastCommandTime;
	RFDCommandPacket received;
	std::unordered_map<CommandID, uint8_t> newCommands;
	std::set<CommandID> oldCommands;

	// Direction handlers
	void handleForward(uint8_t value);
	void handleBackward(uint8_t value);
	void handleLeft(uint8_t value);
	void handleRight(uint8_t value);
	void handleUp(uint8_t value);
	void handleDown(uint8_t value);
	void handlePanLeft(uint8_t value);
	void handlePanRight(uint8_t value);
	void increaseSpeed(uint8_t value);
	void decreaseSpeed(uint8_t value);
	
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
	void executeCommand(CommandID id, uint8_t rawValue);
	void updateRunningCommands();
	void controlTimeouts();

	// Helpers
	void generateCommandMap();
	void applyFailsafeIfTimedOut();
	int16_t mapInput(uint8_t rawValue);
	float smooth(float current, float target, float sensitivity, float deltaTime);

	bool canApplyFailSafe;
	bool canChangeSpeed;

	int movementSpeed;
};

