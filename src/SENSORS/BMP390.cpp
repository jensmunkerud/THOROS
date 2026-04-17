#include "BMP390.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "Status.h"


BMP390::BMP390(Telemetry& tel) : telemetry{tel} {}

void BMP390::begin() {
	if (!bmp.begin_SPI(BMP390_CS)) {
		telemetry.BMP390 = 0;
		return;
	}
	bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
	bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
	bmp.setOutputDataRate(BMP3_ODR_200_HZ);
	telemetry.BMP390 = 1;
}

void BMP390::loop() {
	if (telemetry.BMP390 != 1) {return;}
	if (!bmp.performReading()) {
		telemetry.BMP390 = 0;
		return;
	}
	// float currentPressure = bmp.readPressure();
	// float relativeAltitude = 44330.0 * (1.0 - pow(currentPressure / baselinePressure, 0.5));
	telemetry.altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
	telemetry.pressure = (int)(bmp.pressure * 100); // PRESSURE IN PASCALS
}
