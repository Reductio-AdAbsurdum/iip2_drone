//  ONLY CHANGE VARIABLES HERE!
// BUILD_AS_ANCHOR nicht auskommentiert -> Anchor, falls auskommentiert dann Tag
#define BUILD_AS_ANCHOR
#define UWB_INDEX "0"

// IMPORTS
#include <Wire.h>             // Lib fürs Display & Kommunikation
#include <Adafruit_GFX.h>     // Lib fürs Display
#include <Adafruit_SSD1306.h> // Lib fürs Display
#include <Arduino.h>          // Lib für Arduino Grundfunktionen

#ifdef BUILD_AS_ANCHOR
// placeholder
#else
#include <WiFi.h>
#include <WiFiUdp.h>
#include <MAVLink.h>
#endif

// VARIABLES
// screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define I2C_SCREEN_SDA 39 // esp32 Pins, die für Display (I2C) Kommunikation benutzt werden
#define I2C_SCREEN_SCL 38 // esp32 Pins, die für Display (I2C) Kommunikation benutzt werden

// uwb
#define IO_RXD2 18
#define IO_TXD2 17
#define UWB_RESET 16 // IO16
#define UWB_TAG_COUNT 1
#ifdef BUILD_AS_ANCHOR // Wenn oben BUILD_AS_ANCHOR definiert ist
#define UWB_ROLE "1"   // 1 = Anchor
#else                  // wenn oben BUILD_AS_ANCHOR auskommentiert ist
#define UWB_ROLE "0"   // 0 = Tag
#endif
#define UWB_TAG_COUNT "1"

// LOG serial
#define SERIAL_LOG Serial // die normale USB‑Serielle zum PC
// UWB serial
#define SERIAL_AT mySerial2

// uwb serial object
HardwareSerial SERIAL_AT(2); // Dedizierte serielle Verbindung fürs UWB-Modul, welche UART-Port 2 verwendet

// display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // ein Object namens display wird erzeugt.

// wifi
#ifdef BUILD_AS_ANCHOR
// placeholder
#else
const char *ssid = "DroneTracker_UWB";
const char *password = "password123";
WiFiUDP udp;
const int udpPort = 14550;
IPAddress broadcastIP(192, 168, 4, 255);
#endif

// loop variables
String response = "";
// Timing variables for 4Hz output
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 250; // 250 milliseconds = 4 times per second
// FUNCTIONS

// wifi
#ifdef BUILD_AS_ANCHOR

#else
void send_mavlink_heartbeat()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Tell QGC: "I am a Quadrotor, Active and ready"
  mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  udp.beginPacket(broadcastIP, udpPort);
  udp.write(buf, len);
  udp.endPacket();
}

void send_mavlink_position(float distance_meters)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Base GPS Coordinate (A park in Winterthur, Switzerland)
  int32_t base_lat = 474999500;
  int32_t base_lon = 87375650;

  // 1D Hack: We only move along the Longitude (East/West)
  // 1 meter of distance is roughly 133 units of Longitude at this Latitude
  int32_t current_lat = base_lat;
  int32_t current_lon = base_lon + (distance_meters * 133.0);
  int32_t alt_mm = 0;

  mavlink_msg_global_position_int_pack(1, 1, &msg, millis(), current_lat, current_lon, alt_mm, alt_mm, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  udp.beginPacket(broadcastIP, udpPort);
  udp.write(buf, len);
  udp.endPacket();
}
#endif

// setup(): Einmal beim Start ausgeführt -> Funktion, um die Hardware zu wecken :)
void setup()
{

#ifdef BUILD_AS_ANCHOR
// placeholder
#else
  SERIAL_LOG.print("Creating Wi-Fi Network...");
  // Start the Access Point
  WiFi.softAP(ssid, password);
  SERIAL_LOG.println(" Done!");
  SERIAL_LOG.print("Connect your Mac to Wi-Fi: ");
  SERIAL_LOG.println(ssid);
  SERIAL_LOG.print("ESP32 IP Address: ");
  SERIAL_LOG.println(WiFi.softAPIP());

  // Start listening/sending on the UDP port
  udp.begin(udpPort);
#endif

  // uwb setup
  pinMode(UWB_RESET, OUTPUT);
  digitalWrite(UWB_RESET, HIGH);

  SERIAL_LOG.begin(115200); // startet die USBSerielle zum PC mit Baud 115200

  // Warten bis die Serielle Schnittstelle bereit ist
  while (!SERIAL_LOG)
  {
    delay(10);
  }
  SERIAL_LOG.println("Starting...");

  // Startet die zweite serielle Schnittstelle mit Baud 115200 zu UWB-Modul
  SERIAL_AT.begin(115200, SERIAL_8N1, IO_RXD2, IO_TXD2);
  SERIAL_AT.println("AT");

  // Display Initialisierung -> Startet I2C Display Kommunikation mit den angegebenen Pins
  Wire.begin(I2C_SCREEN_SDA, I2C_SCREEN_SCL);
  delay(1000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) // Hier wird Display initialisiert -> Bei Fehler, Error Meldung
  {
    SERIAL_LOG.println("Display connection failed! Check board settings.");
    while (true)
      ;
  }

  // Schreibt Text aufs Display
  printDisplay();

  sendData("AT?", 2000, 1);
  sendData("AT+RESTORE", 5000, 1); // Reset Modul auf Werkeinstellung
  sendData(config_cmd(), 2000, 1); // config_cmd baut den Konfig-AT-String zusammen inkl. Anchor/Tag-Rolle
  sendData(cap_cmd(), 2000, 1);    // setzt z.B Tag-Anzahl, Report-Frequenz etc.
  sendData("AT+SETRPT=1", 2000, 1);
  sendData("AT+SAVE", 2000, 1);    // Speichert die Konfig im Modul
  sendData("AT+RESTART", 2000, 1); // Startet das Modul mit der neuen Konfig neu
}

// Läuft unendlich oft hintereinander ab, bis der Strom aus ist.
void loop()
{
#ifdef BUILD_AS_ANCHOR
  // Vom PC zum UWB-Modul
  while (SERIAL_LOG.available() > 0) // Läuft bis alle ankommenden Bytes vom PC gelesen wurden
  {
    SERIAL_AT.write(SERIAL_LOG.read()); // Liest ein Byte aus dem Eingangespuffer der PC-Seriellen und schreibt dieses eins zu eins weiter an das UWB Modul.
    yield();
  }

  // Vom UWB-Modul zurück zum PC
  while (SERIAL_AT.available() > 0) // Solange UWB-Modul Daten schickt, wird ausgeführt
  {
    char c = SERIAL_AT.read(); // Liest ein Zeichen aus dem Puffer

    // bis eine Zeilenende (\n) kommt, werden die Strings angehängt
    if (c == '\r') // wenn nur \r kommt, ignorieren.
      continue;
    else if (c == '\n' || c == '\r') // wenn \n
    {
      SERIAL_LOG.println(response); // schickt den bisher gesammelten Text an PC-Monitor

      response = ""; // Löscht den String, damit für die nächste Zeile wieder von vorne gesammelt werden kann
    }
    else // Bis eine Zeilenende (\n) kommt, werden alle Zeichen angehängt (eine Zeile bzw. ein Satz generiert).
      response += c;
  }
#else
  // Read from UWB Module
  while (SERIAL_AT.available() > 0)
  {
    char c = SERIAL_AT.read();

    if (c == '\r')
      continue;
    if (c == '\n')
    {
      if (response.length() > 0)
      {

        // --- THE 4Hz TIMER ---
        if (millis() - lastSendTime >= sendInterval)
        {
          lastSendTime = millis();

          // 1. Send Heartbeat (QGC needs this constantly)
          send_mavlink_heartbeat();

          // 2. Parse the Distance from the UWB string
          int rangeIndex = response.indexOf("range:(");
          if (rangeIndex != -1)
          {
            int start = rangeIndex + 7;
            int comma = response.indexOf(',', start);

            if (comma != -1)
            {
              // Extract distance and convert cm to meters
              float distance_m = response.substring(start, comma).toFloat() / 100.0;

              SERIAL_LOG.print("Distance to Anchor 0: ");
              SERIAL_LOG.print(distance_m);
              SERIAL_LOG.println(" m");

              // 3. Send Fake GPS Location to QGC
              send_mavlink_position(distance_m);
            }
          }
        }
        response = "";
      }
    }
    else
    {
      response += c;
    }
  }
#endif
}

// Funktion: Schreibt Text aufs OLED-Display
void printDisplay()
{
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(12, 0);
  display.println("HSLU IIP2");

  display.setTextSize(1);
  display.setCursor(0, 16);
  display.println("drone tracking v1.0");

  display.setCursor(0, 40);

#ifdef BUILD_AS_ANCHOR
  display.println("A" UWB_INDEX); // Wenn Anchor -> sieht man "A" auf dem Display
#else
  display.println("T" UWB_INDEX); // Wenn Tag -> sieht man "T" auf dem Display
#endif

  display.display();
}

// Funktion: Sendet einen AT-Befehl ans UWB-Modul & wartet eine definierte Zeit auf die Antwort.
String sendData(String command, const int timeout, boolean debug)
{
  String response = "";
  // command = command + "\r\n";

  SERIAL_LOG.println(command);
  SERIAL_AT.println(command); // send the read character to the SERIAL_LOG

  long int time = millis();

  while ((time + timeout) > millis())
  {
    while (SERIAL_AT.available()) // Solange Zeichen von der UWB-Seriellen ankommen, werden sie gelesen
    {

      // The esp has data so display its output to the serial window
      char c = SERIAL_AT.read(); // read the next character.
      response += c;             // hängt das gelesene Zeichen an den Antwort-String an
    }
  }

  if (debug)
  {
    SERIAL_LOG.println(response);
  }

  return response;
}

// Liefert einen zusammengesetzten String zurück
String config_cmd()
{
  return "AT+SETCFG=" UWB_INDEX "," UWB_ROLE ",1,1";
}

// wie config_cmd, baut einen AT-String zusammen
String cap_cmd()
{
  return "AT+SETCAP=" UWB_TAG_COUNT ",10,1";
}