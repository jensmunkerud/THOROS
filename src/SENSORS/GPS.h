#pragma once
#include <TinyGPSPlus.h>
#include <Arduino.h>
#include "Datatypes.h"

constexpr int GPS_RX	{21};
constexpr int GPS_TX	{22};
const int maxPacketLength = 300;

class GPS {
public:
	GPS(Telemetry& tel, DroneState& droneState);
	void loop();

private:
	TinyGPSPlus gps;
	Telemetry& telemetry;
	DroneState& droneState;
	HardwareSerial SerialGPS;
};