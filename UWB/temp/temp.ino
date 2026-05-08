//  ONLY CHANGE VARIABLES HERE!
// BUILD_AS_ANCHOR nicht auskommentiert -> Anchor, falls auskommentiert dann Tag
// #define BUILD_AS_ANCHOR
// UWB_INDEX: 0..3 für Anchors, beliebige Zahl für Tag
#define UWB_INDEX 0

// Stringify-Helper: macht aus der Zahl UWB_INDEX einen String fürs AT-Command
#define _STR(x) #x
#define STR(x) _STR(x)
#define UWB_INDEX_STR STR(UWB_INDEX)

// IMPORTS
#include <Wire.h> // lib for display / communication
#include <Adafruit_GFX.h> // lib for display
#include <Adafruit_SSD1306.h> // lib for display
#include <Arduino.h> // lib for basic arduino

#ifdef BUILD_AS_ANCHOR
// Antennen-Delay Kalibrierung pro Anchor (gemessen mit calibration.ino)
#if UWB_INDEX == 0
#define UWB_ANT_CALIB "16465"
#elif UWB_INDEX == 1
#define UWB_ANT_CALIB "16455"
#elif UWB_INDEX == 2
#define UWB_ANT_CALIB "16450"
#elif UWB_INDEX == 3
#define UWB_ANT_CALIB "16450"
#else
#error "Unbekannter Anchor UWB_INDEX - kein Kalibrierwert definiert"
#endif
#else
#include <MAVLink.h>
#endif


// VARIABLES
// screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define I2C_SCREEN_SDA 39 // IO39
#define I2C_SCREEN_SCL 38 // IO38

// uwb
#define IO_RXD2 18 // IO18
#define IO_TXD2 17 // IO18
#define UWB_RESET 16 // IO16
#define UWB_TAG_COUNT "1"
#ifdef BUILD_AS_ANCHOR // Wenn oben BUILD_AS_ANCHOR definiert ist
#define UWB_ROLE "1"   // 1 = Anchor
#else                  // wenn oben BUILD_AS_ANCHOR auskommentiert ist
#define UWB_ROLE "0"   // 0 = Tag
// ESP32-S3 GPIOs broken out on header J2 pins 5/6.
// IO1 is also a strapping pin (BOOT mode select on S3) -- if the FC pulls it
// LOW at power-on the ESP32 can enter download mode. UART idle is HIGH so this
// is normally fine; add a 10 k pull-up if you see boot issues.
#define IO_FC_RX 2  // ESP32 receives here  -- wire to FC TX
#define IO_FC_TX 1  // ESP32 transmits here -- wire to FC RX
// FC serial object, UART-Port 1
HardwareSerial SERIAL_FC(1);
#endif

// LOG serial, usb to computer
#define SERIAL_LOG Serial
// UWB serial
#define SERIAL_AT mySerial2

// uwb serial object, UART-Port 2
HardwareSerial SERIAL_AT(2);

// display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// wifi
// #ifdef BUILD_AS_ANCHOR
// placeholder
// #else
// const char *ssid = "DroneTracker_UWB";
// const char *password = "password123";
// WiFiUDP udp;
// const int udpPort = 14550;
// IPAddress broadcastIP(192, 168, 4, 255);
// #endif

// --- Filtering (tag only) ---
// Median-of-N on each anchor's distance, run at UWB frame rate. Rejects
// outlier spikes from multipath. Larger N = more rejection but more latency.
#define DIST_FILT_N 5
// EMA on the trilaterated XY position. Lower alpha = smoother but laggier.
// alpha = 1.0 means no smoothing (raw output). 0.3 is a reasonable default.
#define POS_EMA_ALPHA 0.3f

// loop variables
String response = "";
#ifndef BUILD_AS_ANCHOR
// Vision position rate limit: ArduPilot wants >= 5 Hz. We push at most 10 Hz.
unsigned long lastVisionSend = 0;
const unsigned long visionInterval = 100; // 100 ms = 10 Hz
// Heartbeat at 1 Hz, independent of UWB data so the FC always sees us.
unsigned long lastHeartbeat = 0;
const unsigned long heartbeatInterval = 1000;
// EKF origin needs to be set once before the FC accepts vision.
unsigned long lastOriginTime = 0;
uint8_t originSendCount = 0;
// Median filter buffers, one ring per anchor.
float dist_buf[4][DIST_FILT_N] = {{0}};
uint8_t dist_buf_idx = 0;
uint8_t dist_buf_count = 0;
// EMA-smoothed position state.
float xy_smooth[2] = {0.0f, 0.0f};
bool xy_smooth_init = false;
#endif
// FUNCTIONS

#ifndef BUILD_AS_ANCHOR
// =============================================================
// ANCHOR POSITIONS (NED frame, meters) - X/Y only
// =============================================================
// Frame: X = North, Y = East. Origin = your chosen room reference point
// (e.g. the takeoff spot). All 4 anchors are at the same height in our setup,
// so altitude is supplied by the LiDAR rangefinder on the FC, not by UWB.
// Order MUST match the anchor UWB_INDEX.
//
// Measured 2.9 m x 3.0 m rectangle, A0 at origin.
const float ANCHOR_XY[4][2] = {
    // {  X (N) ,  Y (E)  }
    {0.00f, 0.00f}, // Anchor 0
    {2.90f, 0.00f}, // Anchor 1   (2.9 m North of A0)
    {2.90f, 3.00f}, // Anchor 2   (3.0 m East of A1)
    {0.00f, 3.00f}, // Anchor 3   (3.0 m East of A0)
};

// Approximate vertical separation between the tag (on the drone) and the
// anchors (meters, positive = tag higher than anchors). Used to convert UWB
// slant ranges into horizontal distances. Doesn't need to be exact -- a
// rough estimate of typical flight altitude is fine.
#define TAG_HEIGHT_M 1.0f

// MAVLink IDs. SysID = 1 matches the autopilot; CompID 158 = MAV_COMP_ID_PERIPHERAL.
#define MAV_SYS_ID 1
#define MAV_COMP_ID 158

void send_mavlink_heartbeat()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                             MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
                             0, 0, MAV_STATE_ACTIVE);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_FC.write(buf, len);
}

// Sends VISION_POSITION_ESTIMATE so EKF3 fuses our UWB position.
// x_n / y_e are in NED meters relative to the EKF origin.
// z_d is set to 0 here -- altitude comes from the LiDAR (EK3_SRC1_POSZ = 2).
void send_mavlink_vision_position(float x_n, float y_e)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint64_t usec = (uint64_t)micros();

  // If your MAVLink lib's pack signature lacks covariance / reset_counter
  // (older common.xml), drop the trailing  NULL, 0 .
  mavlink_msg_vision_position_estimate_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                            usec, x_n, y_e, 0.0f,
                                            0.0f, 0.0f, 0.0f,
                                            NULL, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_FC.write(buf, len);
}

// Sends SET_GPS_GLOBAL_ORIGIN once -- ArduPilot needs this before it will
// accept VISION_POSITION_ESTIMATE indoors. Lat/lon are arbitrary (the EKF only
// uses the origin as a reference; the room frame is local NED from there).
// You can also do this from your GCS instead and skip calling this function.
void send_set_gps_global_origin()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // HSLU Rotkreuz (~), arbitrary -- only the local frame matters for indoor.
  int32_t lat = 471750000; // degE7
  int32_t lon = 84380000;  // degE7
  int32_t alt = 430000;    // mm AMSL

  mavlink_msg_set_gps_global_origin_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                         1 /* target_system */,
                                         lat, lon, alt,
                                         (uint64_t)micros());
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SERIAL_FC.write(buf, len);
}

// -----------------------------------------------------------------
// Range parser: extracts the 4 anchor distances from a LinkTrack
// "AT+RANGE,...,range:(d0,d1,d2,d3,...),..." frame. Distances in cm.
// Writes meters into out[4]. Returns true if all 4 are valid (>0).
// -----------------------------------------------------------------
bool parse_ranges(const String &line, float out[4])
{
  int rangeIndex = line.indexOf("range:(");
  if (rangeIndex < 0)
    return false;
  int start = rangeIndex + 7;
  int end = line.indexOf(')', start);
  if (end < 0)
    return false;

  String inside = line.substring(start, end);
  int from = 0;
  for (int i = 0; i < 4; i++)
  {
    int comma = inside.indexOf(',', from);
    String tok = (comma < 0) ? inside.substring(from) : inside.substring(from, comma);
    tok.trim();
    long cm = tok.toInt();
    if (cm <= 0)
      return false; // 0 or negative => anchor out of range / not heard
    out[i] = (float)cm / 100.0f;
    if (comma < 0)
      return (i == 3);
    from = comma + 1;
  }
  return true;
}

// -----------------------------------------------------------------
// Median-of-5 helper. 5 elements, in-place tiny bubble sort.
static float median5(float a, float b, float c, float d, float e)
{
  float v[5] = {a, b, c, d, e};
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4 - i; j++)
      if (v[j] > v[j + 1])
      {
        float t = v[j];
        v[j] = v[j + 1];
        v[j + 1] = t;
      }
  return v[2];
}

// -----------------------------------------------------------------
// 2D trilateration for coplanar anchors.
//
// Step 1: convert UWB slant ranges to horizontal distances using the assumed
//         vertical separation TAG_HEIGHT_M:   d_h = sqrt(d_slant^2 - h^2)
//
// Step 2: subtract the anchor-0 sphere equation from anchors 1..3 to get a
//         linear system in (X, Y) only:
//             2*(A_i - A_0) . p_xy = d0h^2 - dih^2 + |A_i|^2 - |A_0|^2
//         3 equations, 2 unknowns -> overdetermined. We solve it in the
//         least-squares sense via the 2x2 normal equations  (A^T A) x = A^T b.
//         The 4th anchor effectively averages out range noise.
//
// Returns false on collinear-anchor geometry (singular A^T A).
// -----------------------------------------------------------------
bool trilaterate_2d(const float dist[4], float xy_out[2])
{
  // Slant -> horizontal range. Clamp to 0 if slant is shorter than our height
  // assumption (would otherwise make d_h^2 negative).
  float dh[4];
  const float h_sq = TAG_HEIGHT_M * TAG_HEIGHT_M;
  for (int i = 0; i < 4; i++)
  {
    float dh_sq = dist[i] * dist[i] - h_sq;
    dh[i] = (dh_sq > 0.0f) ? sqrtf(dh_sq) : 0.0f;
  }

  float A[3][2];
  float b[3];
  float A0_sq = ANCHOR_XY[0][0] * ANCHOR_XY[0][0] + ANCHOR_XY[0][1] * ANCHOR_XY[0][1];
  float dh0_sq = dh[0] * dh[0];

  for (int i = 0; i < 3; i++)
  {
    int ai = i + 1;
    A[i][0] = 2.0f * (ANCHOR_XY[ai][0] - ANCHOR_XY[0][0]);
    A[i][1] = 2.0f * (ANCHOR_XY[ai][1] - ANCHOR_XY[0][1]);
    float Ai_sq = ANCHOR_XY[ai][0] * ANCHOR_XY[ai][0] + ANCHOR_XY[ai][1] * ANCHOR_XY[ai][1];
    b[i] = dh0_sq - dh[ai] * dh[ai] + Ai_sq - A0_sq;
  }

  // Normal equations on a 2x2: ATA = A^T A, ATb = A^T b.
  float ATA00 = 0, ATA01 = 0, ATA11 = 0;
  float ATb0 = 0, ATb1 = 0;
  for (int i = 0; i < 3; i++)
  {
    ATA00 += A[i][0] * A[i][0];
    ATA01 += A[i][0] * A[i][1];
    ATA11 += A[i][1] * A[i][1];
    ATb0 += A[i][0] * b[i];
    ATb1 += A[i][1] * b[i];
  }

  float det = ATA00 * ATA11 - ATA01 * ATA01;
  if (fabsf(det) < 1e-6f)
    return false; // collinear anchors

  xy_out[0] = (ATA11 * ATb0 - ATA01 * ATb1) / det;
  xy_out[1] = (-ATA01 * ATb0 + ATA00 * ATb1) / det;
  return true;
}
#endif

// setup(): Einmal beim Start ausgeführt -> Funktion, um die Hardware zu wecken :)
void setup()
{

#ifdef BUILD_AS_ANCHOR
#else
  // SERIAL_LOG.print("Creating Wi-Fi Network...");
  // Start the Access Point
  // WiFi.softAP(ssid, password);
  // SERIAL_LOG.println(" Done!");
  // SERIAL_LOG.print("Connect your Mac to Wi-Fi: ");
  // SERIAL_LOG.println(ssid);
  // SERIAL_LOG.print("ESP32 IP Address: ");
  // SERIAL_LOG.println(WiFi.softAPIP());

  // Start listening/sending on the UDP port
  // udp.begin(udpPort);
  // Start the hardware connection to the Flight Controller
  SERIAL_FC.begin(115200, SERIAL_8N1, IO_FC_RX, IO_FC_TX);
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
#ifdef BUILD_AS_ANCHOR
  // Antennen-Delay aus Kalibrierung setzen (Wert pro Anchor oben definiert)
  sendData("AT+SETANT=" UWB_ANT_CALIB, 2000, 1);
#endif
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
  // Heartbeat at 1 Hz, independent of UWB activity. This is what makes the FC
  // (and Mission Planner via the FC) see compid 158 even before UWB data flows.
  if (millis() - lastHeartbeat >= heartbeatInterval)
  {
    lastHeartbeat = millis();
    send_mavlink_heartbeat();
    SERIAL_LOG.println("[mavlink] heartbeat sent");
  }

  // Send SET_GPS_GLOBAL_ORIGIN a few times at startup so the FC will accept
  // vision updates. Skip this if you're setting the origin from the GCS instead.
  if (originSendCount < 5 && (millis() - lastOriginTime > 2000))
  {
    send_set_gps_global_origin();
    lastOriginTime = millis();
    originSendCount++;
    SERIAL_LOG.printf("[mavlink] SET_GPS_GLOBAL_ORIGIN (%u/5)\n", originSendCount);
  }

  // Read one full line from the UWB module, then process it.
  while (SERIAL_AT.available() > 0)
  {
    char c = SERIAL_AT.read();

    if (c == '\r')
      continue;
    if (c == '\n')
    {
      if (response.length() > 0)
      {
        // DEBUG: dump every line we get from the UWB module so we can see what
        // it actually says. Comment this out once ranging works to reduce noise.
        SERIAL_LOG.print("[uwb] ");
        SERIAL_LOG.println(response);

        // Try to parse a range frame. Not every line is one.
        float dist_m[4];
        if (parse_ranges(response, dist_m))
        {
          // Push every frame into the median ring buffer (don't rate-limit
          // this -- more samples = better outlier rejection).
          for (int i = 0; i < 4; i++)
            dist_buf[i][dist_buf_idx] = dist_m[i];
          dist_buf_idx = (dist_buf_idx + 1) % DIST_FILT_N;
          if (dist_buf_count < DIST_FILT_N)
            dist_buf_count++;

          // Rate-limit the trilateration + send to 10 Hz, and wait until the
          // median window is full so the first few frames don't leak through.
          if (dist_buf_count >= DIST_FILT_N &&
              millis() - lastVisionSend >= visionInterval)
          {
            lastVisionSend = millis();

            // Median-filter each anchor's distance.
            float dist_filt[4];
            for (int i = 0; i < 4; i++)
              dist_filt[i] = median5(dist_buf[i][0], dist_buf[i][1],
                                     dist_buf[i][2], dist_buf[i][3],
                                     dist_buf[i][4]);

            float xy_raw[2];
            if (trilaterate_2d(dist_filt, xy_raw))
            {
              // EMA smoothing: seed on first valid fix, then blend.
              if (!xy_smooth_init)
              {
                xy_smooth[0] = xy_raw[0];
                xy_smooth[1] = xy_raw[1];
                xy_smooth_init = true;
              }
              else
              {
                xy_smooth[0] = POS_EMA_ALPHA * xy_raw[0] +
                               (1.0f - POS_EMA_ALPHA) * xy_smooth[0];
                xy_smooth[1] = POS_EMA_ALPHA * xy_raw[1] +
                               (1.0f - POS_EMA_ALPHA) * xy_smooth[1];
              }

              SERIAL_LOG.printf("d=[%.2f %.2f %.2f %.2f] m  raw=(%.2f, %.2f)  smooth=(%.2f, %.2f)\n",
                                dist_filt[0], dist_filt[1], dist_filt[2], dist_filt[3],
                                xy_raw[0], xy_raw[1], xy_smooth[0], xy_smooth[1]);
              send_mavlink_vision_position(xy_smooth[0], xy_smooth[1]);
            }
            else
            {
              SERIAL_LOG.println("Trilateration failed: anchor geometry singular.");
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

// =============================================================
// ArduCopter 4.6.3 setup (set on the FC, NOT here):
//
//   AHRS_EKF_TYPE    = 3      // Use EKF3
//   EK3_ENABLE       = 1
//   EK3_SRC1_POSXY   = 6      // ExternalNav (UWB) for horizontal position
//   EK3_SRC1_POSZ    = 2      // RangeFinder (LiDAR) for altitude
//   EK3_SRC1_VELXY   = 0      // No external velocity
//   EK3_SRC1_VELZ    = 0
//   EK3_SRC1_YAW     = 1      // Compass for yaw
//   VISO_TYPE        = 1      // Enable visual odometry input
//   GPS_TYPE         = 0      // No GPS indoors
//   ARMING_CHECK     = ...    // May need to disable GPS arming check for indoor
//
// LiDAR rangefinder must be configured separately:
//   RNGFND1_TYPE     = ...    // Match your sensor (e.g. 8 = LightWare I2C, etc.)
//   RNGFND1_ORIENT   = 25     // Down-facing
//
// MAVLink uplink to this ESP32 (whichever SERIAL port it's wired to):
//   SERIALx_PROTOCOL = 2      // MAVLink2
//   SERIALx_BAUD     = 115    // 115200
//
// EKF origin: set automatically by send_set_gps_global_origin() above, OR
// from the GCS (Mission Planner / QGC: right-click map -> Set Home).
// =============================================================

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
  display.println("A" UWB_INDEX_STR); // anchor -> "A" on display
#else
  display.println("T" UWB_INDEX_STR); // tag -> "T" on display
#endif

  display.display();
}

// helper function for sending AT-Commands from the esp32 to the UWB module
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

// config command for uwb
String config_cmd()
{
  return "AT+SETCFG=" UWB_INDEX_STR "," UWB_ROLE ",1,1";
}

// cap command for uwb
String cap_cmd()
{
  return "AT+SETCAP=" UWB_TAG_COUNT ",10,1";
}