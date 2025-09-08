#include <Arduino.h>
#include <sbus.h>
#include <TinyGPS++.h>
#include <QMC5883LCompass.h>
#include <PID_v1.h>
#include <PWMServo.h>
#include <math.h>

// Constants
const uint32_t PID_INTERVAL_US = 2000; // 500hz
const uint32_t SBUS_INTERVAL_US = 9000; // 111.11hz
const uint32_t LIDAR_TIMEOUT_MS = 100;

// Dropper Servo
PWMServo dropperServo;
int servoPin = 2;

// SBUS
bfs::SbusRx sbus_rx(&Serial1);
bfs::SbusTx sbus_tx(&Serial1);
elapsedMicros sbusTimer;

// GPS & Compass
TinyGPSPlus gps;
QMC5883LCompass compass;

// LIDAR
uint8_t buffer[16]; // Number of bytes for TOFsense Protocol
float lastLidarDistance = 0;
unsigned long lastLidarUpdate = 0;
bool lidarStale = false;

// Altitude
const int hoverThrottle = 737.8;
double takeoffGPSAltitude = 0;
double currentAltitude = 0;
double targetAltitude = 0;
double flightAltitude = 30; // Ajust this to tell the drone how high to fly (m)

// Position
double currentLatitude = 0;
double currentLongitude = 0;
double targetLatitude = 0;
double targetLongitude = 0;

// Stored Positions
double pos1_lat = 0;
double pos1_lon = 0;
double pos2_lat = 0;
double pos2_lon = 0;
bool pos1Stored = false;
bool pos2Stored = false;

// PID Controllers
elapsedMicros pidTimer;
float maxAngle = 20; // Max angle in degress the drone will tilt to. Bigger angles will make drone faster but will also make it more prone to ossolations
double altitudeOutput = 0;
double latitudeOutput = 0;
double longitudeOutput = 0;
// Tune Kp, Ki, and Kd for each controller to get the desired responsiveness and reduse ossolations
PID altitudePID(&currentAltitude, &altitudeOutput, &targetAltitude, 20.0, 1.0, 50.0, DIRECT);
PID latitudePID(&currentLatitude, &latitudeOutput, &targetLatitude, 1.0, 0.003, 1, DIRECT); // 1, 0.002, 1
PID longitudePID(&currentLongitude, &longitudeOutput, &targetLongitude, 1.0, 0.003, 1, DIRECT);

// Debug Flag
bool debug = true;

void setup() {
  if (debug) SerialUSB.begin(115200); // Serial monitor for debugging
  Serial1.begin(100000); // SBUS
  Serial4.begin(921600); // LIDAR
  Serial5.begin(115200); // GPS

  sbus_rx.Begin();
  sbus_tx.Begin();

  dropperServo.attach(servoPin, 1000, 2000);

  // Initilise and calibrate compass
  compass.init();
  compass.setCalibrationOffsets(-22.00, -369.00, 495.00);
  compass.setCalibrationScales(0.98, 0.93, 1.11);
  compass.setMode(0x01, 0x0C, 0x10, 0x00);

  // Clamp PID outputs
  double sbusAngle = maxAngle / 0.05487804878; // Convert max angle into sbus
  altitudePID.SetOutputLimits(-40, 40);
  latitudePID.SetOutputLimits(sbusAngle * -1, sbusAngle);
  longitudePID.SetOutputLimits(sbusAngle * -1, sbusAngle);
}

void readGPS() {
  // Get GPS Info
  while (Serial5.available()) {
    gps.encode(Serial5.read());
  }
  if (gps.location.isValid()) {
    // Multiply the lat and lon by 1000000 to get it in a range that the pid controller can handle
    currentLatitude = gps.location.lat() * 1000000;
    currentLongitude = gps.location.lng() * 1000000;
  }
}

void readLIDAR() {
  // Get LIDAR info
  while (Serial4.available()) {
    if (Serial4.peek() == 0x57) { // Look for start byte without discarding any bytes
      while (Serial4.available() < 16); // Wait for the rest of the packet
      Serial4.readBytes(buffer, 16); // Store and discard bytes
      uint32_t distance = buffer[8] | (buffer[9] << 8) | (buffer[10] << 16); // Rearrage bytes 8, 9, 10 from little-endian form and combine them into a single number
      lastLidarDistance = distance / 1000.0; // Convert from mm to m
      lastLidarUpdate = millis(); 
      lidarStale = false;
      break; // Break loop
    } else {
      Serial4.read(); // Discard byte
    }
  }
  // Timer to check whether the LIDAR info is still relevant
  if (millis() - lastLidarUpdate > LIDAR_TIMEOUT_MS) {
    lidarStale = true;
  }
}

float computeCorrectedAltitude(int pitchCmd, int rollCmd) {
  // Get true distance from ground
  // Convert roll and pitch commands into degrees that the drone will tilt
  float pitchAngle = ((float)(pitchCmd - 992) / 820.0) * 45.0;
  float rollAngle = ((float)(rollCmd - 992) / 820.0) * 45.0;
  // Convert to radians
  float pitchRad = radians(pitchAngle);
  float rollRad = radians(rollAngle);
  float correction = cos(pitchRad) * cos(rollRad); // Get correction factor
  return lastLidarDistance * correction; // Apply correction to get true distance from ground
}

void updateAltitude(int pitchCmd, int rollCmd) {
  float GPSAltitude = gps.altitude.meters()-takeoffGPSAltitude;
  float lidarAltitude = computeCorrectedAltitude(pitchCmd, rollCmd);
  if (!lidarStale && lidarAltitude < 20) {
    // If LIDAR is valid use that
    currentAltitude = lidarAltitude;
  } else if (gps.altitude.isValid()) {
    // Else use gps to get altitude
    currentAltitude = GPSAltitude;
  }
}

void updatePID() {
  // Run Pid Controllers
  altitudePID.Compute();
  latitudePID.Compute();
  longitudePID.Compute();
}

void altitudeHold(bfs::SbusData &sbusData) {
  if (sbusData.ch[5] < 1900 && !lidarStale) {
    // Enable altitude hold
    if (sbusData.ch[7] < 500) { 
      altitudePID.SetMode(AUTOMATIC); // Turn on PID controller
      if (currentAltitude < 0.20) {
        // If landed set throttle to 0
        sbusData.ch[2] = 173;
      } else {
        targetAltitude = 0.12;
        // Send PID output to throttle channel
        sbusData.ch[2] = constrain(hoverThrottle + altitudeOutput, 172, 1811);
      }
    } else if (sbusData.ch[7] < 1200 && sbusData.ch[7] > 500) {
      altitudePID.SetMode(AUTOMATIC); // Turn on PID controller
      targetAltitude = flightAltitude + 0.175; // Account for the height of the landing legs
      // Send PID output to throttle channel
      sbusData.ch[2] = constrain(hoverThrottle + altitudeOutput, 172, 1811);
    } else {
      // Turn off altitude PID controller to stop intergral building
      altitudePID.SetMode(MANUAL);
    }
  }
}

void storePosition1(bfs::SbusData &sbusData) {
  if (sbusData.ch[4] > 1200 && gps.location.isValid() && !pos1Stored) {
    pos1_lat = gps.location.lat() * 1000000;
    pos1_lon = gps.location.lng() * 1000000;
    takeoffGPSAltitude = gps.altitude.meters();
    pos1Stored = true;
  }
}

void getHeadingStorePosition2(bfs::SbusData &sbusData) {
  compass.read();
  int heading = compass.getAzimuth() - 30;
  if (heading <= 5 && heading >= -5 && sbusData.ch[6] <= 992 && gps.location.isValid() && !pos2Stored) {
    pos2_lat = gps.location.lat() * 1000000;
    pos2_lon = gps.location.lng() * 1000000;
    pos2Stored = true;
    sbusData.ch[6] = 172;
  } else {
    sbusData.ch[6] = 1811;
  }
}

void goToStoredPosition(bfs::SbusData &sbusData) {
  if (sbusData.ch[5] < 1200 && sbusData.ch[5] > 500 && pos1Stored) {
    // Go to pos 1
    // Enable the PID Controllers
    latitudePID.SetMode(AUTOMATIC);
    longitudePID.SetMode(AUTOMATIC);
    // Set target to pos 1
    targetLatitude = pos1_lat;
    targetLongitude = pos1_lon;
    // Send PID Output to the roll and pitch channels
    sbusData.ch[1] = constrain(992 - latitudeOutput, 172, 1811);
    sbusData.ch[0] = constrain(992 - longitudeOutput, 172, 1811);
    if (debug) SerialUSB.println("Going to Position 1");

  } else if (sbusData.ch[5] < 500 && pos2Stored) {
    // Go to pos 2
    // Enable the PID Controllers
    latitudePID.SetMode(AUTOMATIC);
    longitudePID.SetMode(AUTOMATIC);
    // Set target to pos 2
    targetLatitude = pos2_lat;
    targetLongitude = pos2_lon;
    // Send PID Output to the roll and pitch channels
    sbusData.ch[1] = constrain(992 - latitudeOutput, 172, 1811);
    sbusData.ch[0] = constrain(992 - longitudeOutput, 172, 1811);
    if (debug) SerialUSB.println("Going to Position 2");
  } else {
    // Manual Control (Angle Mode)
    // Disable the PID controllers to stop the intergral term from building
    latitudePID.SetMode(MANUAL);
    longitudePID.SetMode(MANUAL);
  }
}

void dropPayload(bfs::SbusData &sbusData) {
  if (sbusData.ch[7] < 1200 && sbusData.ch[7] > 500) {
    // Go to 180 degrees
    dropperServo.write(180);
  } else if (sbusData.ch[7] > 1200 || sbusData.ch[7] < 500) {
    // Go to 0 degrees
    dropperServo.write(0);
  }
}

void loop() {
  // Get GPS and LIDAR info
  readGPS();
  readLIDAR();

  if (sbus_rx.Read()) {
    bfs::SbusData sbusData = sbus_rx.data();

    updateAltitude(sbusData.ch[1], sbusData.ch[0]);

    storePosition1(sbusData);
    getHeadingStorePosition2(sbusData);
    dropPayload(sbusData);

    if (pidTimer > PID_INTERVAL_US) {
      updatePID();
      altitudeHold(sbusData);
      goToStoredPosition(sbusData);
      pidTimer = 0;
    }

    if (sbusTimer > SBUS_INTERVAL_US) {
      sbus_tx.data(sbusData);
      sbus_tx.Write();
      sbusTimer = 0;
    }
    if (debug) {
      // Print debug info in the serial monitor
      SerialUSB.print("Pitch Out: "); SerialUSB.println(sbusData.ch[1]);
      SerialUSB.print("Roll Out: "); SerialUSB.println(sbusData.ch[0]);
      SerialUSB.print("Target Altitude: "); SerialUSB.println(targetAltitude);
      SerialUSB.print("Target Latitude: "); SerialUSB.println(targetLatitude, 6);
      SerialUSB.print("Target Longitude: "); SerialUSB.println(targetLongitude, 6);
      SerialUSB.print("Current Latitude: "); SerialUSB.println(currentLatitude, 6);
      SerialUSB.print("Current Longitude: "); SerialUSB.println(currentLongitude, 6);
      SerialUSB.print("Sats: "); SerialUSB.println(gps.satellites.value());
      SerialUSB.print("Alt Error: "); SerialUSB.println(currentAltitude - targetAltitude);
      SerialUSB.print("Alt PID Out: "); SerialUSB.println(altitudeOutput);
      SerialUSB.print("Alt current: "); SerialUSB.println(currentAltitude);
      SerialUSB.print("Alt target: "); SerialUSB.println(targetAltitude);
      SerialUSB.print("Throttle Out:"); SerialUSB.println(sbusData.ch[2]);
      SerialUSB.print("GPS Alt"); SerialUSB.println(gps.altitude.meters());
    }
  }
}