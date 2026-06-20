#pragma once
#include <cstdint>
#include <unordered_map>
#include <functional>
#include <chrono>
#include "Datatypes.h"
#include "COMMS/RFD900.h"
#include "set"

static constexpr float PAN_SPEED					{5.0f};		// [deg/s]
static constexpr float MAX_PAN						{175.0f};	// [deg]
static constexpr float TILT_SPEED					{10.0f};	// [deg/s]
static constexpr float MAX_TILT_ANGLE				{5.0f};		// [deg]
static constexpr float THROTTLE_SPEED				{400.0f};	// [u/s]
static constexpr float MAX_THROTTLE					{1200.0f};	// [u]
static constexpr int   MOVEMENT_TIMEOUT_MS			{35};		// [ms]
static constexpr int   ARM_HOLD_MS					{1000};		// [ms]


// Command IDs matching radio protocol
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
	SPEED_UP	= 109,
	SPEED_DOWN	= 110,
	LOG_TOGGLE	= 245,
	ARM			= 235,
	KILL		= 254,
};

class MovementController {
public:
	MovementController(Telemetry& tel, Drone& drone, RFD900& rfd90);
	void begin();
	void update();
	bool isToggled;
	double Kp = 1, Ki = 0, Kd = 1;
	void clearInputs(bool force = false);

private:
	FlightControls target;
	Telemetry& telemetry;
	Drone& drone;
	RFD900& rfd900;
	std::unordered_map<CommandID, std::function<void(uint8_t)>> commandMap;
	std::chrono::steady_clock::time_point lastCommandTime;
	RFDCommandPacket received;
	std::unordered_map<CommandID, uint8_t> newCommands;
	std::set<CommandID> runningCommands;

	// Control handlers
	void handleForward(uint8_t value);
	void handleBackward(uint8_t value);
	void handleLeft(uint8_t value);
	void handleRight(uint8_t value);
	void handleUp(uint8_t value);
	void handleDown(uint8_t value);
	void handlePanLeft(uint8_t value);
	void handlePanRight(uint8_t value);
	void handleArm(uint8_t value);
	void increaseSpeed(uint8_t value);
	void decreaseSpeed(uint8_t value);
	
	void toggle(uint8_t value);
	void log_toggle(uint8_t value);

	long lastTime{0};
	long deltaMs{0};

	// Arm button hold state
	std::chrono::steady_clock::time_point armPressedAt;
	bool armButtonActive{false};
	void executeCommand(CommandID id, uint8_t rawValue);
	void updateRunningCommands();
	void controlTimeouts();

	// Helpers
	void generateCommandMap();
	void applyFailsafeIfTimedOut();
	void updateFlightMode(const FlightControls& controls);
	bool hasLateralInput(const FlightControls& controls) const;
	bool hasYawInput(const FlightControls& controls) const;
	int16_t mapInput(uint8_t rawValue);
	float smooth(float current, float target, float sensitivity, float deltaTime);

	bool canApplyFailSafe;
	bool canChangeSpeed;

	int movementSpeed;
};

