#pragma once
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "Status.h"

constexpr int BMP390_CS {9};

class BMP390 {
	public:
	BMP390(Status& status);
	void begin();
	void loop();

	private:
	Status& status;
};