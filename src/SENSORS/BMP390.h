#pragma once
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "Status.h"

constexpr int SEALEVELPRESSURE_HPA {1010};
constexpr int BMP390_CS {4};

class BMP390 {
	public:
	BMP390(Telemetry& telemetry);
	void begin();
	void loop();

	private:
	Telemetry& telemetry;
	Adafruit_BMP3XX bmp;
};