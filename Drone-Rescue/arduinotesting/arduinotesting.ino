#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_LSM303.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Define variables
double x_setpoint, y_setpoint, x_input, y_input, x_output, y_output;
float x_kp, x_ki, x_kd, y_kp, y_ki, y_kd;
#define GPSRXPin 4
#define GPSTXPin 3
#define GPSBaud 115200

// Setup pid controllers
PID x_pid(&x_input, &output, &x_setpoint, kp, ki, kd, DIRECT);
PID y_pid(&x_input, &output, &y_setpoint, kp, ki, kd, DIRECT);

void setup() {

  // Initilise linked variables
  input = analogRead(0);
  x_setpoint = 100;
  y_setpoint = 100;

  // Activate pid controllers
  x_pid.SetMode(AUTOMATIC);
  y_pid.SetMode(AUTOMATIC);
}

void loop() {

  // Update input
  x_input = gps_x;
  y_input = gps_y;

  // Run pid controllers
  x_pid.Compute();
  y_pid.Compute();

  // Apply correctional ajustments
  analogWrite(3, x_output);
  analogWrite(4, y_output);
}

int c_location() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    lsm.read();
    
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
    float heading = lsm.magHeading();
  }
  return 
}