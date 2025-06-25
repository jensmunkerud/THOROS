#include <Arduino.h>
#include <SPI.h>
#include "stabilizer.h"
#include "COMMS/NRF24.h"
#include "COMMS/LORA.h"


// PARAMETERS
constexpr int speed {100};	// Vehicle speed
constexpr int P {1};		// Proportional parameter
constexpr int I {1};		// Integral parameter
constexpr int D {1};		// Derivative parameter

// LORA sends "Power on" when its gucci gang setup yaknow

// CONFIGURATION
constexpr int motorL {13};
constexpr int motorR {14};
constexpr int motorB {15};
unsigned long prev = 0;
const unsigned long interval = 1000;
// NRF24 nrf24;
LORA lora;

void onMessageReceived(const char* msg) {
	Serial.print("Received: ");
	Serial.println(msg);
}


// ----- //
// SETUP //
// ----- //
void setup() {
	Serial1.begin(9600);
	delay(2000);
	// nrf24.begin();
	// nrf24.setCallback(onMessageReceived);
}


// ---- //
// LOOP //
// ---- //
void loop() {
	unsigned long current = millis();
	// nrf24.loop();
	lora.loop();
	
	if (current - prev >= interval) {
		prev = current;
	}
}

void freeze() {

}

void emergencyLand() {

}

// Turn off EVERYTHING
void lobotomize() {

}

