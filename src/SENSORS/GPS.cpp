#include "GPS.h"

GPS::GPS(Status& status) : status{status}, SerialGPS(1)
{
	SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}


void GPS::loop() {
	// FETCH GPS DATA
	while (SerialGPS.available() > 0) {
		gps.encode(SerialGPS.read());
	}

	if (gps.location.isValid()) {
		status.latitude  = gps.location.lat() * 1e7;
		status.longitude = gps.location.lng() * 1e7;
		Serial.print("Lat: ");
		Serial.print(gps.location.lat(), 6);
		Serial.print(" | Lon: ");
		Serial.println(gps.location.lng(), 6);
	} else {
		Serial.println("Waiting for GPS fix...");
	}

}
