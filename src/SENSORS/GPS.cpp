#include "GPS.h"

GPS::GPS(Status& status) : status{status}
// , gpsSerial(2, 3) 
{
	pinMode(GPS_IN, INPUT);
}


void GPS::loop() {
	// // FETCH GPS DATA
	// while (gpsSerial.available() > 0) {
	// 	gps.encode(gpsSerial.read());
	// }

	// if (gps.location.isValid()) {
	// 	status.latitude  = gps.location.lat() * 1e7;
	// 	status.longitude = gps.location.lng() * 1e7;
	// }
	// if (digitalRead(GPS_IN) == LOW) {
	// 	delay(maxPacketLength); 

	// }
}


// void loop() {

//   for (int i = 0; i < 500; i++) // read 500 byte in data buffer
// 								// if there are less data
// 								// handle it with your code
// 				// if packet length is constant and known, 
// 					// change 500 bytes to what is needed 				
//   {
// 	c[i] = 0; // clear byte i from old data

// 	// wait for start bit
// 	while (digitalRead(GPS_IN))
// 	{
// 	  ;
// 	}
// 	// GPS_IN is LOW -> begin of start bit 
	
// 	delayMicroseconds(150); // wait 150 µs to be in the middle of data bit0
// 							// 9600 baud rate -> bit length is 104 µs. 
// 							// 104 µs * 1,5 = 156 µs -> allow 6 µs for code
// 							// execution
// 	c[i] = c[i] | digitalRead(GPS_IN); // read first data bit in buffer c[0]
// 	delayMicroseconds(100);           // wait for next data bit; allow 4 µs for code.
// 	c[i] = c[i] | digitalRead(GPS_IN) << 1;
// 	delayMicroseconds(100);           // the same for all remaining bits
// 	c[i] = c[i] | digitalRead(GPS_IN) << 2;
// 	delayMicroseconds(100);
// 	c[i] = c[i] | digitalRead(GPS_IN) << 3;
// 	delayMicroseconds(100);
// 	c[i] = c[i] | digitalRead(GPS_IN) << 4;
// 	delayMicroseconds(100);
// 	c[i] = c[i] | digitalRead(GPS_IN) << 5;
// 	delayMicroseconds(100);
// 	c[i] = c[i] | digitalRead(GPS_IN) << 6;
// 	delayMicroseconds(100);
// 	c[i] = c[i] | digitalRead(GPS_IN) << 7;
// 	delayMicroseconds(100);
//   }
  
//  Serial.println(String(c)); // print out data buffer containing 500 byte data

// // fill in code for your application
// }
