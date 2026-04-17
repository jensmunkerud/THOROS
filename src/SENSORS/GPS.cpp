#include "GPS.h"

GPS::GPS(Telemetry& tel, DroneState& droneState) : 
telemetry{tel},
droneState{droneState},
SerialGPS(1)
{
	SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}


void GPS::loop() {
	// FETCH GPS DATA
	while (SerialGPS.available() > 0) {
		gps.encode(SerialGPS.read());
	}

	if (gps.location.isValid()) {
		telemetry.gpsFix = 1;
		telemetry.latitude  = gps.location.lat() * 1e7;
		telemetry.longitude = gps.location.lng() * 1e7;
	} else {
		telemetry.gpsFix = 0;
	}

}
