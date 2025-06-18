// Import librays
#include <sbus.h>
#include <Servo.h>

//***********************************************************************************************//

// Toggle Serial Printing Of Data For Debugging (Uncomment to Enable)

// PWM values from RX
// #define PWM_IN_VALUES

//***********************************************************************************************//

// Defines

// Define # of RX PWM outputs (servos/motors) Max outputs for r81 v2 is 8
const int noPWMRX = 6;
Servo servos[noPWMRX];

// Define SBUS RX (Pin 0 for SBUS RX on Teensy 4.0)
bfs::SbusRx sbusRx(&Serial1, true);
bfs::SbusData sbusData;

//***********************************************************************************************//

void setup() {
    SerialUSB.begin(115200); // Debug Serial Monitor
    Serial1.begin(100000, SERIAL_8E2); // SBUS Serial in
    sbusRx.Begin();

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH); // Turn on Teensy 4.0 LED

    // Attach servos to pins
    for (int i = 0; i < noPWMRX; i++) {
        servos[i].attach(i + 1);
    }

    SerialUSB.println("SBUS to PWM initialized!");
}

//***********************************************************************************************//

void loop() {
    if (sbusRx.Read()) { // Process SBUS data
        sbusData = sbusRx.data();

        #ifdef PWM_IN_VALUES
            SerialUSB.print("PWM Outputs: ");
        #endif

        for (int i = 0; i < noPWMRX; i++) { // Limit to only used channels
            int pwmValue = map(sbusData.ch[i], 172, 1811, 1000, 2000); // Convert SBUS to PWM
            servos[i].writeMicroseconds(pwmValue); // Send PWM signal to servos

            #ifdef PWM_IN_VALUES
                SerialUSB.print(pwmValue); // Print PWM value
                SerialUSB.print(" ");
            #endif
        }

        #ifdef PWM_IN_VALUES
            SerialUSB.println();
        #endif

        digitalWrite(13, !digitalRead(13)); // Blink LED to show data updates (Blink speed depends on update rate below)
    }

    delay(1); // Update rate (ms)
}