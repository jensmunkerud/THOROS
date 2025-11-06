#include "BMP390.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "Status.h"

// constexpr int SEALEVELPRESSURE_HPA {1013};

Adafruit_BMP3XX bmp;

BMP390::BMP390(Status& status) : status{status} {}

void BMP390::begin() {
	if (!bmp.begin_SPI(BMP390_CS)) {
		status.BMP390 = 0;
		return;
	}
	bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
	bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
	bmp.setOutputDataRate(BMP3_ODR_200_HZ);
	status.BMP390 = 1;
}

void BMP390::loop() {
	if (status.BMP390 != 1) {return;}
	if (!bmp.performReading()) {
		status.BMP390 = 0;
		status.pressure = 69;
		return;
	}
	// float currentPressure = bmp.readPressure();
	// float relativeAltitude = 44330.0 * (1.0 - pow(currentPressure / baselinePressure, 0.5));
	status.pressure = bmp.pressure;
	// Serial.println(status.pressure);
}
