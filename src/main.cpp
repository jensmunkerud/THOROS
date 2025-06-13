#include <Arduino.h>
#include "stabilizer.h"
#include "transceiver.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


// PARAMETERS
constexpr int speed {100};	// Vehicle speed
constexpr int P {1};		// Proportional parameter
constexpr int I {1};		// Integral parameter
constexpr int D {1};		// Derivative parameter

// CONFIGURATION
constexpr int motorL {13};
constexpr int motorR {14};
constexpr int motorB {15};
// RF24 radio(CE_PIN, CSN_PIN);

Transceiver transceiver;

void onMessageReceived(const char* msg) {
	Serial.print("Received in main yeey: ");
	Serial.println(msg);
}


void setup() {
	Serial.begin(9600);
	// noInterrupts(); // suppposed to make arduino run faster... disables millis... research this
	// https://www.youtube.com/watch?v=hRqJkfB8uoE
	Serial.println("Good morning");
	transceiver.setCallback(onMessageReceived);
}

unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 second in ms

void loop() {
		transceiver.loop();
	unsigned long currentMillis = millis();

	if (currentMillis - previousMillis >= interval) {
		previousMillis = currentMillis;
		transceiver.send("this is sent from controller");
	}
}

void freeze() {

}

void emergencyLand() {

}

// Turn off EVERYTHING
void lobotomize() {

}

