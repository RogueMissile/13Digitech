#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  Serial5.begin(115200); 
}

void loop() {
  while (Serial5.available() > 0) {
    gps.encode(Serial5.read());
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    Serial.println("\n--- GPS Data ---");

    if (gps.location.isValid()) {
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());

      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);

      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);

      Serial.print("Altitude (m): ");
      Serial.println(gps.altitude.meters());

      Serial.print("Speed (km/h): ");
      Serial.println(gps.speed.kmph());

    } else {
      Serial.println("No valid location fix.");
    }

    Serial.println("----------------");
  }
}