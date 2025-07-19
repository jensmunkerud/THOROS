#pragma once
#include <Wire.h>
#include <SPI.h>
#include <ICM_20948.h>
#include "Status.h"

constexpr int ICM20948_CS {8};

class ICM20948 {
	public:
	ICM20948(Status& stat);
	ICM_20948_SPI icm20948;
	void begin();
	void loop();
	bool State;

	private:
	Status& status;
};
