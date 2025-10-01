#include <Arduino.h>
#include <SPI.h>
#include "stabilizer.h"
#include "COMMS/NRF24.h"
#include "COMMS/LORA.h"
#include "SENSORS/ICM20948.h"
#include "SENSORS/BMP390.h"
#include "Status.h"

// PARAMETERS
constexpr int speed {100};	// Vehicle speed
constexpr int P {1};		// Proportional parameter
constexpr int I {1};		// Integral parameter
constexpr int D {1};		// Derivative parameter
constexpr unsigned long interval = 1000/60;
constexpr unsigned long interval2 = 1000/10;


// CONFIGURATION
constexpr int motor1 {3};
constexpr int motor2 {4};
constexpr int motor3 {2};
constexpr int motor4 {5};

unsigned long prev = 0;
unsigned long prev2 = 0;

Status status;
ICM20948 icm20948(status);
BMP390 bmp390(status);



int flyHigh = 0;
void onMessageReceived(const int command, const int value) {
	if (command == 100) {
		flyHigh = 1;
	}
	if (command == 110 || command == 111) {
		flyHigh = 0;
	}
	if (command == 101) {
		flyHigh = -1;
	}
}


// ----------------- //
//       SETUP       //
// ----------------- //
void setup() {
	Serial.begin(115200);
	pinMode(7, OUTPUT);
	// pinMode(8, OUTPUT);
	digitalWrite(7, HIGH);
	// digitalWrite(8, HIGH);
	delay(2000);
	icm20948.begin();
	bmp390.begin();
	nrf24.begin();
	nrf24.setCallback(onMessageReceived);
}


// ---------------- //
//       LOOP       //
// ---------------- //
void loop() {
	unsigned long current = millis();
	// lora.loop();
	nrf24.loop();
	
	if (current - prev >= interval) {
		prev = current;
		icm20948.loop();
		bmp390.loop();
	}
	if (current - prev2 >= interval2) {
		prev2 = current;
		status.altitude += flyHigh;
	}
}


void freeze() {

}

void emergencyLand() {

}

// Turn off EVERYTHING
void lobotomize() {

}

