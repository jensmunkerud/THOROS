#pragma once
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "Status.h"

constexpr int SEALEVELPRESSURE_HPA {1010};
constexpr int BMP390_CS {4};

class BMP390 {
	public:
	BMP390(Status& status);
	void begin();
	void loop();

	private:
	Status& status;
	Adafruit_BMP3XX bmp;
};