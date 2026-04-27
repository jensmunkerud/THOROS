#pragma once
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "MISC/Datatypes.h"

constexpr int SEALEVELPRESSURE_HPA {1010};
constexpr int BMP390_CS {4};
constexpr int BMP390_SCK {18};
constexpr int BMP390_MISO {19};
constexpr int BMP390_MOSI {23};

class BMP390 {
public:
	BMP390(Telemetry& telemetry, Drone& drone);
	void begin();
	void loop();

private:
	Telemetry& telemetry;
	Drone& drone;
	Adafruit_BMP3XX bmp;
	SPIClass vspi;
};