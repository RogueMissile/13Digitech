uint8_t buffer[16];

void setup() {
  Serial.begin(115200);      // Serial Monitor
  Serial4.begin(921600);     // ToFSense F2P
}

void loop() {
  // Look for packet start: 0x57
  while (Serial4.available()) {
    if (Serial4.peek() == 0x57) {
      while (Serial4.available() < 16); // Wait for full packet

      Serial4.readBytes(buffer, 16);

      // Extract distance (bytes 8, 9, 10)
      uint32_t distance = buffer[8] | (buffer[9] << 8) | (buffer[10] << 16);
      float distance_m = distance / 1000.0;

      // Print to Serial Monitor
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.print(" mm (");
      Serial.print(distance_m, 3);
      Serial.println(" m)");

      break; // exit loop after one valid packet
    } else {
      Serial4.read(); // discard byte and keep searching
    }
  }
}