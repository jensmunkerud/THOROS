// #include <TinyGPSPlus.h>

// TinyGPSPlus gps;

// void setup() {
//   Serial.begin(115200);         // Serial Monitor
//   Serial1.begin(9600);          // GPS module usually runs at 9600 baud
//   while (!Serial);
//   Serial.println("GPS Starting...");
// }

// void loop() {
//   while (Serial1.available() > 0) {
// 	gps.encode(Serial1.read());

// 	if (gps.location.isUpdated()) {
// 	  Serial.print("Latitude: ");
// 	  Serial.println(gps.location.lat(), 6);
// 	  Serial.print("Longitude: ");
// 	  Serial.println(gps.location.lng(), 6);
// 	}
//   }
// }
