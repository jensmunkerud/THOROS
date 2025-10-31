#pragma once
#include <TinyGPSPlus.h>
#include <Arduino.h>
// #include "SoftwareSerial.h"
#include "Status.h"

constexpr int GPS_OUT {3};
constexpr int GPS_IN {4};
const int maxPacketLength = 300; // 300 µs; this applies to my GPS GT-U7.
								 // you have to determin the max packet length
								 // of your device
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