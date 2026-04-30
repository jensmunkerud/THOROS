#include "GPS.h"

GPS::GPS(Telemetry& tel, Drone& drone) : 
	telemetry{tel},
	drone{drone},
	SerialGPS{GPS_SERIAL}
{
	SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}


void GPS::loop() {
	// FETCH GPS DATA
	while (SerialGPS.available() > 0) {
		gps.encode(SerialGPS.read());
	}

	if (gps.location.isValid()) {
		drone.GPS_OK = true;
		TelemetryLockGuard telemetryLock(telemetry);
		telemetry.latitude  = gps.location.lat() * 1e7;
		telemetry.longitude = gps.location.lng() * 1e7;
	} else {
		drone.GPS_OK = false;
	}

}
