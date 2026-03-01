#pragma once
#include <Wire.h>
#include <SPI.h>
#include <ICM_20948.h>
#include "Status.h"
#include "Madgwick.h"

static constexpr int ICM20948_CS {15};
static constexpr ICM_20948_smplrt_t ICM_SAMPLERATE {1000};

class ICM20948 {
	public:
	ICM20948(Status& status);
	ICM_20948_SPI icm20948;
	void begin();
	void loop();

	private:
	Status& status;
	icm_20948_DMP_data_t data;
	double q1;
	double q2;
	double q3;
	double q0;
	Madgwick filter;
	ICM_20948_AGMT_t agmt;
	uint32_t lastTime;
};
