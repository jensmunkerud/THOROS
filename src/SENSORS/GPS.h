#pragma once
#include <TinyGPSPlus.h>
#include <Arduino.h>
#include "Status.h"

constexpr int GPS_OUT	{22};
constexpr int GPS_IN	{21};
const int maxPacketLength = 300;

class GPS {
	public:
	GPS(Status& status);
	void loop();


	private:
	TinyGPSPlus gps;
	Status& status;
	// SoftwareSerial gpsSerial;
	char c[500]; // data buffer
};