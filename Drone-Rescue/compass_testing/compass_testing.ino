#include <QMC5883LCompass.h>

QMC5883LCompass compass;

void setup() {
  Serial.begin(115200);
  compass.init();
	compass.setCalibrationOffsets(-163.00, -578.00, 125.00);
	compass.setCalibrationScales(1.02, 0.99, 0.99);
	compass.setMode(0x01, 0x0C, 0x10, 0x00);
}

void loop() {

	int x, y, z, a, b;
	char myArray[3];
	
	compass.read();
  
	x = compass.getX();
	y = compass.getY();
	z = compass.getZ();
	
	a = compass.getAzimuth();
	
	b = compass.getBearing(a);

	compass.getDirection(myArray, a);
  
  
	Serial.print("X: ");
	Serial.print(x);

	Serial.print(" Y: ");
	Serial.print(y);

	Serial.print(" Z: ");
	Serial.print(z);

	Serial.print(" Azimuth: ");
	Serial.print(a);

	Serial.print(" Bearing: ");
	Serial.print(b);

	Serial.print(" Direction: ");
	Serial.print(myArray[0]);
	Serial.print(myArray[1]);
	Serial.print(myArray[2]);

	Serial.println();

	delay(250);
}