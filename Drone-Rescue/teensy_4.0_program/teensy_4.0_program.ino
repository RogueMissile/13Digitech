#include <PID_v1.h>
#include <sbus.h>
#include <Arduino.h>

bfs::SbusRx sbus_rx(&Serial1);   // Receiver on Serial1 rx
bfs::SbusTx sbus_tx(&Serial1);   // Flight controller on Serial1 tx

const uint32_t SBUS_INTERVAL_US = 9000; // Approx. 111Hz transmission
elapsedMicros sbus_timer;               // Microsecond counter

void setup() {
  
  // Initilise serial communications with Receiver and Flight Controller
  Serial1.begin(100000); 
  sbus_rx.Begin();
  sbus_tx.Begin();
}

void loop() {
  if (sbus_rx.Read()) {
    // Get incoming SBUS data
    bfs::SbusData sbusData = sbus_rx.data();

    // Modify SBUS data
    // sbusData.ch[0] = 1500;

    // Transmit SBUS data at controlled intervals
    if (sbus_timer > SBUS_INTERVAL_US) {
      sbus_tx.data(sbusData);
      sbus_tx.Write();
      sbus_timer = 0;
    }
  }
}