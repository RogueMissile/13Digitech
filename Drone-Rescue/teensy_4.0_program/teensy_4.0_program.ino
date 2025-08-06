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
bool headfreeResetDone = false;

// PID variables
double currentAltitude = 0;
double targetAltitude = 0;
double pidOutput = 0;
double throttleInput = 0;

// PID objects
PID altitudePID(&currentAltitude, &pidOutput, &targetAltitude, 18.0, 0.5, 1.0, DIRECT);

// Altitude hold state flags
bool altitudeHoldActive = false;
bool altitudeBoosted = false;

// Hover throttle value
const int hoverThrottle = 691;

void setup() {
  SerialUSB.begin(115200); // Initilise connecton with serial moniter

  Serial1.begin(100000); // Initilise SBUS connecton 
  sbus_rx.Begin();
  sbus_tx.Begin();

  Serial5.begin(115200); // Initilise GPS connection

  compass.init();
	compass.setCalibrationOffsets(-22.00, -369.00, 495.00);
  compass.setCalibrationScales(0.98, 0.93, 1.11);
	compass.setMode(0x01, 0x0C, 0x10, 0x00);

  altitudePID.SetMode(AUTOMATIC);
  altitudePID.SetOutputLimits(-600, 600); // Altitude PID limits
}

void loop() {
  while (Serial5.available() > 0) {
    gps.encode(Serial5.read());
  }

  if (gps.location.isValid()) {
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
    int heading = compass.getAzimuth();
    heading -= 30;
    

    // Override CH7 when heading is north
    if (heading <= 5 && heading >= -5) {
      sbusData.ch[6] = 172;
      SerialUSB.println("Heading is North — CH7 set to 172");
    }


    if (ch6 < 1200) {
      if (!altitudeHoldActive) {
        targetAltitude = currentAltitude;
        altitudeHoldActive = true;
        altitudeBoosted = false;
      }

      // CH6 < 1200 → Boost altitude
      if (ch6 < 500 && !altitudeBoosted) {
        targetAltitude += 5.0;
        altitudeBoosted = true;
      }

      // CH6 > 1200 → Revert boost
      if (ch6 > 500 && altitudeBoosted) {
        targetAltitude -= 5.0;
        altitudeBoosted = false;
      }

      altitudePID.Compute();

      int adjustedThrottle = constrain(hoverThrottle + pidOutput, 172, 1811);
      sbusData.ch[2] = adjustedThrottle;

    } else {
      altitudeHoldActive = false;
      altitudeBoosted = false;
    }

    // Print target altitude
      SerialUSB.println("targetAltitude");
      SerialUSB.println(targetAltitude);
    if (sbus_timer > SBUS_INTERVAL_US) {
      
      sbus_tx.data(sbusData);
      sbus_tx.Write();
      sbus_timer = 0;
    }
  }
}