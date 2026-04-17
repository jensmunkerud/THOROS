#pragma once
#include <TinyGPSPlus.h>
#include <Arduino.h>
#include "Status.h"

constexpr int GPS_RX	{21};
constexpr int GPS_TX	{22};
const int maxPacketLength = 300;

class GPS {
	public:
	GPS(Telemetry& tel);
	void loop();

	private:
	TinyGPSPlus gps;
	Telemetry& telemetry;
	HardwareSerial SerialGPS;
};