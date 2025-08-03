#include <Arduino.h>
#include <sbus.h>
#include <TinyGPS++.h>
#include <QMC5883LCompass.h>
#include <HardwareSerial.h>
#include <PID_v1.h>

// SBUS setup
bfs::SbusRx sbus_rx(&Serial1);   // Receiver on Serial1 rx
bfs::SbusTx sbus_tx(&Serial1);   // Flight controller on Serial1 tx

const uint32_t SBUS_INTERVAL_US = 9000; // Approx. 111Hz transmission
elapsedMicros sbus_timer;               // Microsecond counter

// GPS setup
TinyGPSPlus gps; // Create GPS object

// PID variables
double currentAltitude = 0;
double targetAltitude = 0;
double pidOutput = 0;
double throttleInput = 0;

// PID tuning (adjust as needed)
PID altitudePID(&currentAltitude, &pidOutput, &targetAltitude, 2.0, 0.5, 1.0, DIRECT);

// Altitude hold state
bool altitudeHoldActive = false;

// Hover throttle value
const int hoverThrottle = 691;

void setup() {
  // Serial monitor
  SerialUSB.begin(115200);

  // SBUS communication
  Serial1.begin(100000); 
  sbus_rx.Begin();
  sbus_tx.Begin();

  // GPS communication
  Serial5.begin(115200);

  // PID setup
  altitudePID.SetMode(AUTOMATIC);
  altitudePID.SetOutputLimits(-300, 300); // Adjust based on throttle sensitivity
}

void loop() {
  // Read GPS data
  while (Serial5.available() > 0) {
    gps.encode(Serial5.read());
  }

  // If GPS location is valid, update current altitude
  if (gps.location.isValid()) {
    currentAltitude = gps.altitude.meters();

    /*SerialUSB.print("Satellites: ");
    SerialUSB.println(gps.satellites.value());

    SerialUSB.print("Latitude: ");
    SerialUSB.println(gps.location.lat(), 6);

    SerialUSB.print("Longitude: ");
    SerialUSB.println(gps.location.lng(), 6);

    SerialUSB.print("Altitude (m): ");
    SerialUSB.println(currentAltitude);

    SerialUSB.print("Speed (km/h): ");
    SerialUSB.println(gps.speed.kmph());*/
  }

  // Read SBUS data
  if (sbus_rx.Read()) {
    bfs::SbusData sbusData = sbus_rx.data();

    int ch6 = sbusData.ch[5]; // Channel 6

    if (ch6 < 1200) {
      if (!altitudeHoldActive) {
        targetAltitude = currentAltitude; // Lock target altitude
        altitudeHoldActive = true;
        SerialUSB.println("Altitude Hold Activated");
      }

      altitudePID.Compute();

      // Apply PID output to throttle (Channel 3)
      int adjustedThrottle = constrain(hoverThrottle + pidOutput, 172, 1811);
      sbusData.ch[2] = adjustedThrottle;

      SerialUSB.print("PID Output: ");
      SerialUSB.println(pidOutput);
      SerialUSB.print("Throttle: ");
      SerialUSB.println(adjustedThrottle);
    } else {
      altitudeHoldActive = false; // Disable altitude hold
    }

    // Transmit SBUS data at regular intervals
    if (sbus_timer > SBUS_INTERVAL_US) {
      sbus_tx.data(sbusData);
      sbus_tx.Write();
      sbus_timer = 0;
    }
  }
}