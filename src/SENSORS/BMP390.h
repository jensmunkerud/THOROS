#pragma once
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "Datatypes.h"

constexpr int SEALEVELPRESSURE_HPA {1010};
constexpr int BMP390_CS {4};

class BMP390 {
public:
	BMP390(Telemetry& telemetry, DroneState& droneState);
	void begin();
	void loop();

private:
	Telemetry& telemetry;
	DroneState& droneState;
	Adafruit_BMP3XX bmp;
};