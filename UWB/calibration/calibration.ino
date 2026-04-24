Anchor 0: AT+SETANT=16465
Anchor 1: AT+SETANT=16455
Anchor 2: AT+SETANT=16450
Anchor 3: AT+SETANT=16450

#include <Arduino.h>

#define IO_RXD2 18
#define IO_TXD2 17
#define UWB_RESET 16

HardwareSerial SERIAL_AT(2);

// Filter variables
String incomingLine = "";
unsigned long lastRangePrintTime = 0;
const unsigned long rangePrintInterval = 3000; // Prints distance every 3 seconds

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\n--- CALIBRATION MODE (FILTERED) ---");
  Serial.println("Distances will print every 3 seconds.");
  Serial.println("Type your AT+SETANT commands below.");
  
  // --- HARDWARE RESET SEQUENCE (Keeps the chip awake!) ---
  pinMode(UWB_RESET, OUTPUT);
  digitalWrite(UWB_RESET, LOW);
  delay(200);
  digitalWrite(UWB_RESET, HIGH);

  SERIAL_AT.begin(115200, SERIAL_8N1, IO_RXD2, IO_TXD2);
}

void loop() {
  // --- Read from UWB module and filter ---
  while (SERIAL_AT.available()) {
    char c = SERIAL_AT.read();
    
    // Build the string until a newline character is received
    if (c == '\n') {
      // Check if the line is a distance reading
      if (incomingLine.startsWith("AT+RANGE") || incomingLine.indexOf("range:(") != -1) {
        // Only print if 3 seconds have passed
        if (millis() - lastRangePrintTime >= rangePrintInterval) {
          Serial.println(incomingLine);
          lastRangePrintTime = millis();
        }
      } 
      // If it's anything else (like an "OK" or boot text), print immediately
      else {
        if (incomingLine.length() > 0) {
          Serial.println(incomingLine);
        }
      }
      // Reset the line variable
      incomingLine = ""; 
    } 
    // Ignore carriage returns
    else if (c != '\r') {
      incomingLine += c; 
    }
  }
  
  // --- Read from PC and send to UWB module ---
  while (Serial.available()) {
    SERIAL_AT.write(Serial.read());
  }
}