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
		status.gpsFix = 1;
		status.latitude  = gps.location.lat() * 1e7;
		status.longitude = gps.location.lng() * 1e7;
	} else {
		status.gpsFix = 0;
	}

}
