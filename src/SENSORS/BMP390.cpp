#include "BMP390.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "Datatypes.h"


BMP390::BMP390(Telemetry& tel, Drone& drone) : 
telemetry{tel},
drone{drone}
{}

void BMP390::begin() {
	if (!bmp.begin_SPI(BMP390_CS)) {
		drone.PRESSURE_OK = false;
		return;
	}
	bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
	bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
	bmp.setOutputDataRate(BMP3_ODR_200_HZ);
	drone.PRESSURE_OK = true;
}

void BMP390::loop() {
	if (!drone.PRESSURE_OK) {return;}
	if (!bmp.performReading()) {
		drone.PRESSURE_OK = false;
		return;
	}
	// float currentPressure = bmp.readPressure();
	// float relativeAltitude = 44330.0 * (1.0 - pow(currentPressure / baselinePressure, 0.5));
	telemetry.altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
	telemetry.pressure = (int)(bmp.pressure * 100); // PRESSURE IN PASCALS
}
