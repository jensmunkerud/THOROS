#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>

// Use hardware SPI and define the CS pin
constexpr int BMP_CS {10};
Adafruit_BMP3XX bmp;

constexpr int SEALEVELPRESSURE_HPA {1013.25};

void setup() {
	Serial.begin(115200);
	while (!Serial);

	Serial.println("BMP390 SPI Test");

	if (!bmp.begin_SPI(BMP_CS)) {
		Serial.println("Could not find a valid BMP3 sensor, check wiring!");
		while (1);
	}

	bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
	bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
	bmp.setOutputDataRate(BMP3_ODR_50_HZ);

	Serial.println("BMP390 Initialized over SPI.");
}

void loop() {
	if (!bmp.performReading()) {
		Serial.println("Failed to perform reading :(");
		return;
	}

	Serial.print("Temperature = ");
	Serial.print(bmp.temperature);
	Serial.println(" *C");

	Serial.print("Pressure = ");
	Serial.print(bmp.pressure / 100.0);
	Serial.println(" hPa");

	Serial.print("Approx. Altitude = ");
	Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
	Serial.println(" m");

	Serial.println();
	delay(1000);
}
