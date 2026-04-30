#pragma once
#include <TinyGPSPlus.h>
#include <Arduino.h>
#include "MISC/Datatypes.h"

constexpr int GPS_RX	{21};
constexpr int GPS_TX	{22};
constexpr uint8_t GPS_SERIAL {1};
const int maxPacketLength = 300;

class GPS {
public:
	GPS(Telemetry& tel, Drone& drone);
	void loop();

private:
	TinyGPSPlus gps;
	Telemetry& telemetry;
	Drone& drone;
	HardwareSerial SerialGPS;
};