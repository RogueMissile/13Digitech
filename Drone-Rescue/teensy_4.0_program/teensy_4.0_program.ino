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

QMC5883LCompass compass;
int referenceHeading = 0;

// Altitude PID variables
double currentAltitude = 0;
double targetAltitude = 0;
double pidOutput = 0;

// Position PID variables
double currentLatitude = 0;
double currentLongitude = 0;
double storedLatitude = 0;
double storedLongitude = 0;
double targetLatitude = 0;
double targetLongitude = 0;
double latitudeOutput = 0;
double longitudeOutput = 0;

// PID objects
PID altitudePID(&currentAltitude, &pidOutput, &targetAltitude, 18.0, 0.5, 1.0, DIRECT);
PID latitudePID(&currentLatitude, &latitudeOutput, &targetLatitude, 1.0, 0.002, 1.0, DIRECT);
PID longitudePID(&currentLongitude, &longitudeOutput, &targetLongitude, 1.0, 0.002, 1.0, DIRECT);

// State flags
bool altitudeHoldActive = false;
bool positionHoldActive = false;
bool destinationGoToActive = false;

// Hover throttle value
const int hoverThrottle = 691;

void setup() {
  SerialUSB.begin(115200); // Serial monitor
  Serial1.begin(100000);   // SBUS
  sbus_rx.Begin();
  sbus_tx.Begin();

  Serial5.begin(115200);   // GPS

  compass.init();
  compass.setCalibrationOffsets(-22.00, -369.00, 495.00);
  compass.setCalibrationScales(0.98, 0.93, 1.11);
  compass.setMode(0x01, 0x0C, 0x10, 0x00);

  altitudePID.SetMode(AUTOMATIC);
  altitudePID.SetOutputLimits(-600, 600); // Altitude PID limits

  latitudePID.SetMode(AUTOMATIC);
  latitudePID.SetOutputLimits(-500, 500); // Latitude PID limits

  longitudePID.SetMode(AUTOMATIC);
  longitudePID.SetOutputLimits(-500, 500); // Longitude PID limits
}

void loop() {
  while (Serial5.available() > 0) {
    gps.encode(Serial5.read());
  }

  if (gps.location.isValid()) {
    currentLatitude = gps.location.lat()*1000000;
    currentLongitude = gps.location.lng()*1000000;
    currentAltitude = gps.altitude.meters();
  }

  if (sbus_rx.Read()) {
    bfs::SbusData sbusData = sbus_rx.data();

    int ch1 = sbusData.ch[0]; // Channel 1 (Roll) 
    int ch2 = sbusData.ch[1]; // Channel 2 (Pitch) 
    int ch3 = sbusData.ch[2]; // Channel 3 (Throttle) 
    int ch4 = sbusData.ch[3]; // Channel 4 (Yaw) 
    int ch5 = sbusData.ch[4]; // Channel 5 (Arm) 
    int ch6 = sbusData.ch[5]; // Channel 6 (Flight Mode) 
    int ch7 = sbusData.ch[6]; // Channel 7 (Aux 1) 
    int ch8 = sbusData.ch[7]; // Channel 8 (Aux 2)

    compass.read();
    int heading = compass.getAzimuth() - 30;

    sbusData.ch[7] = 1811;

    // Override CH7 when heading is north
    if (heading <= 5 && heading >= -5 && ch8 <= 1500) {
      sbusData.ch[7] = 172;
      SerialUSB.println("Heading is North — CH7 set to 172");

      if (gps.location.isValid()) {
        storedLatitude = gps.location.lat()*1000000;
        storedLongitude = gps.location.lng()*1000000;
      }
    }

    /*if (ch6 < 1200) {
      // ALTITUDE HOLD
      if (!altitudeHoldActive) {
        targetAltitude = currentAltitude + 5;
        altitudeHoldActive = true;
      }

      altitudePID.Compute();
      int adjustedThrottle = constrain(hoverThrottle + pidOutput, 172, 1811);
      sbusData.ch[2] = adjustedThrottle;
    } else {
      altitudeHoldActive = false;
    }*/

    // POSITION HOLD
    if (ch6 < 1200 && ch6 > 500) {
      if (!positionHoldActive) {
        targetLatitude = currentLatitude;
        targetLongitude = currentLongitude;
        positionHoldActive = true;
      }

      latitudePID.Compute();
      longitudePID.Compute();

      sbusData.ch[1] = constrain(992 - latitudeOutput, 172, 1811); // Pitch (Latitude)
      sbusData.ch[0] = constrain(992 - longitudeOutput, 172, 1811); // Roll (Longitude)

    } else if (ch6 < 500) {
      if (!destinationGoToActive) {
        targetLatitude = storedLatitude;
        targetLongitude = storedLongitude;
        destinationGoToActive = true;
      }

      latitudePID.Compute();
      longitudePID.Compute();

      sbusData.ch[1] = constrain(992 - latitudeOutput, 172, 1811); // Pitch (Latitude)
      sbusData.ch[0] = constrain(992 - longitudeOutput, 172, 1811); // Roll (Longitude)

    } else {
      positionHoldActive = false;
      destinationGoToActive = false;
    }


    // Debug output
    SerialUSB.print("Pitch Out: "); SerialUSB.println(sbusData.ch[1]);
    SerialUSB.print("Roll Out: "); SerialUSB.println(sbusData.ch[0]);
    SerialUSB.print("Target Altitude: "); SerialUSB.println(targetAltitude);
    SerialUSB.print("Target Latitude: "); SerialUSB.println(targetLatitude, 6);
    SerialUSB.print("Target Longitude: "); SerialUSB.println(targetLongitude, 6);
    SerialUSB.print("Current Latitude: "); SerialUSB.println(currentLatitude, 6);
    SerialUSB.print("Current Longitude: "); SerialUSB.println(currentLongitude, 6);
    SerialUSB.print("Sats: "); SerialUSB.println(gps.satellites.value());

    if (sbus_timer > SBUS_INTERVAL_US) {
      sbus_tx.data(sbusData);
      sbus_tx.Write();
      sbus_timer = 0;
    }
  }
}