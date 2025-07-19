#include <Arduino.h>
#include <SPI.h>
#include "stabilizer.h"
#include "COMMS/NRF24.h"
#include "COMMS/LORA.h"
#include "SENSORS/ICM20948.h"
#include "Status.h"

// PARAMETERS
constexpr int speed {100};	// Vehicle speed
constexpr int P {1};		// Proportional parameter
constexpr int I {1};		// Integral parameter
constexpr int D {1};		// Derivative parameter
constexpr unsigned long interval = 1000/60;

// LORA sends "Power on" when its gucci gang setup yaknow

// CONFIGURATION

constexpr int motorFL {13};
constexpr int motorFR {14};
constexpr int motorBL {15};
constexpr int motorBR {16};

unsigned long prev = 0;
// LORA lora;
Status status;
ICM20948 icm20948(status);
NRF24 nrf24(status);

void onMessageReceived(const char* msg) {
	Serial.print("Received: ");
	Serial.println(msg);
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
	nrf24.begin();
	nrf24.setCallback(onMessageReceived);
}


// ---------------- //
//       LOOP       //
// ---------------- //
void loop() {
	unsigned long current = millis();
	// lora.loop();
	
	if (current - prev >= interval) {
		prev = current;
		icm20948.loop();
		// nrf24.loop();
		nrf24.sendStatus();
	}
}


void freeze() {

}

void emergencyLand() {

}

// Turn off EVERYTHING
void lobotomize() {

}

