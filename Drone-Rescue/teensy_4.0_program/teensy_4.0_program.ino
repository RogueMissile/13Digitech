#include <Arduino.h>
#include <sbus.h>
#include <TinyGPS++.h>
#include <QMC5883LCompass.h>
#include <HardwareSerial.h>
#include <PID_v1.h>

bfs::SbusRx sbus_rx(&Serial1);   // Receiver on Serial1 rx
bfs::SbusTx sbus_tx(&Serial1);   // Flight controller on Serial1 tx

const uint32_t SBUS_INTERVAL_US = 9000; // Approx. 111Hz transmission
elapsedMicros sbus_timer;               // Microsecond counter

TinyGPSPlus gps; // Create GPS object

int hoverThrottle = -1;  // Stores the recorded throttle value
bool throttleCaptured = false;  // Flag to prevent repeated capture


void setup() {
  // Initilise serial communication with serial moniter
  SerialUSB.begin(115200);
  // Initilise serial communications with Receiver and Flight Controller
  Serial1.begin(100000); 
  sbus_rx.Begin();
  sbus_tx.Begin();
  // Initilise serial communication with GPS
  Serial5.begin(115200);
}

void loop() {
  while (Serial5.available() > 0) {
    gps.encode(Serial5.read());
  }

  /*if (gps.location.isValid()) {
    SerialUSB.print("Satellites: ");
    SerialUSB.println(gps.satellites.value());

    SerialUSB.print("Latitude: ");
    SerialUSB.println(gps.location.lat(), 6);

    SerialUSB.print("Longitude: ");
    SerialUSB.println(gps.location.lng(), 6);

    SerialUSB.print("Altitude (m): ");
    SerialUSB.println(gps.altitude.meters());

    SerialUSB.print("Speed (km/h): ");
    SerialUSB.println(gps.speed.kmph());
  }*/

  if (sbus_rx.Read()) {
    bfs::SbusData sbusData = sbus_rx.data();

    // Capture throttle when CH7 goes high (>1200)
    if (sbusData.ch[6] > 1200 && !throttleCaptured) {
      hoverThrottle = sbusData.ch[2];  // Record throttle (CH3)
      throttleCaptured = true;
      SerialUSB.print("Hover throttle recorded: ");
      SerialUSB.println(hoverThrottle);
    }

    // Reset capture flag if CH7 goes low
    if (sbusData.ch[6] <= 1200) {
      throttleCaptured = false;
    }

    // Print recorded throttle when CH6 goes high (>1200)
    if (sbusData.ch[5] > 1200 && hoverThrottle != -1) {
      SerialUSB.print("Stored hover throttle: ");
      SerialUSB.println(hoverThrottle);
    }

    // Transmit SBUS data at controlled intervals
    if (sbus_timer > SBUS_INTERVAL_US) {
      sbus_tx.data(sbusData);
      sbus_tx.Write();
      sbus_timer = 0;
    }
  }
}
