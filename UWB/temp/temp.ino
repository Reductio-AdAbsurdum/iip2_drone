//  ONLY CHANGE VARIABLES HERE!
// BUILD_AS_ANCHOR nicht auskommentiert -> Anchor, falls auskommentiert dann Tag
// #define BUILD_AS_ANCHOR
// UWB_INDEX: 0..3 für Anchors, beliebige Zahl für Tag
#define UWB_INDEX 0

// ENABLE_WIFI_DEBUG: 1 = WiFi SoftAP + UDP debug stream to laptop plotter,
//                    0 = no WiFi at all (radio fully off).
// Disable when chasing power / brownout issues -- WiFi TX bursts pull
// 350-500 mA peaks that sag the ESP's supply rail and cause MAVLink
// heartbeats to drop on the FC side. MAVLink to the FC is unaffected
// either way; only the laptop-side debug data goes away.
#define ENABLE_WIFI_DEBUG 0

// VERBOSE_UWB_LOG: 1 = echo every UWB line and per-VPE-send debug to USB
//                  Serial. HEAVY -- ~10 KB/s of USB writes at fps=66.
//                  0 = quiet (default).
// With no USB host attached during flight, blocking writes can stall the
// loop, drop UART bytes from the UWB module, and starve MAVLink
// heartbeats / VPE. Only enable when actively debugging on USB.
#define VERBOSE_UWB_LOG 0

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
#if ENABLE_WIFI_DEBUG
#include <WiFi.h>
#include <WiFiUdp.h>
#endif
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

// wifi (tag only) -- SoftAP so the laptop joins this ESP32 and listens on UDP
#if !defined(BUILD_AS_ANCHOR) && ENABLE_WIFI_DEBUG
const char *ssid = "DroneTracker_UWB";
const char *password = "drone1234";
WiFiUDP udp;
const int udpPort = 14550;
IPAddress broadcastIP(192, 168, 4, 255);
#endif

// --- Filtering (tag only) ---
// EKF3 on the FC does the position smoothing (it has the IMU). On the tag we
// only do two things: kill range-level multipath spikes, and kill (x, y)
// fixes that imply impossible jumps. No EMA -- pre-smoothing here would
// just lag the data the EKF expects to be observation-time-accurate.
//
// Median-of-N on each anchor's distance. Rejects single-frame range outliers.
// Larger N = more rejection, more latency. 5 @ 10 Hz UWB = 500 ms window.
#define DIST_FILT_N 5
// Velocity gate on the trilaterated (x, y). Reject any fix whose jump from
// the last accepted fix exceeds this. At slow indoor flight (~0.5 m/s) and
// 10 Hz updates, real motion is ~0.05 m per frame -- 0.5 m is 10x that, so
// this only triggers on gross multipath / NLOS spikes, never real motion.
#define POS_GATE_M 0.5f
// If the gate rejects this many fixes in a row, accept the next one anyway.
// Recovery path for: drone picked up and moved, EKF reset, long NLOS stretch.
#define POS_GATE_MAX_REJECT 10

// --- Per-anchor diagnostics window for the UDP debug stream ---
// Rolling window used for computing std-dev of each anchor's raw range. 20
// samples @ 10 Hz = 2 s window -- short enough to react to a degrading anchor,
// long enough that single spikes don't dominate the metric.
#define STATS_WIN_N 20
// Per-frame spike threshold: a frame is flagged for anchor i if
// |d_raw[i] - median5(buffer[i])| > SPIKE_THRESH_M. 10 cm is well above the
// ~2-3 cm UWB noise floor but below typical multipath spikes.
#define SPIKE_THRESH_M 0.10f

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
// Per-anchor stats ring (raw distances over a longer window) for std-dev.
// Zeros mean "anchor not heard on that frame" -- the std calc skips them.
float stats_buf[4][STATS_WIN_N] = {{0}};
uint8_t stats_idx = 0;
uint8_t stats_count = 0;
// UWB frame rate (Hz), recomputed roughly once per second.
unsigned long fps_window_start = 0;
uint16_t fps_counter = 0;
float current_fps = 0.0f;

// Cached responses to the AT+GET queries run once at boot. Rebroadcast over
// UDP every modinfoInterval so a laptop that joined the SoftAP late still
// receives the module's state.
String modinfo_ver = "";
String modinfo_cfg = "";
String modinfo_ant = "";
String modinfo_cap = "";
String modinfo_pow = "";
String modinfo_rpt = "";
unsigned long lastModinfo = 0;
const unsigned long modinfoInterval = 5000; // 5 s
// Velocity-gate state: last accepted (x, y) and consecutive-reject counter.
float xy_last[2] = {0.0f, 0.0f};
bool xy_last_init = false;
uint8_t pos_gate_rejects = 0;
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
// UDP debug stream to the laptop plotter (uwb_plot.py).
// Emitted on every UWB frame.
//
// CSV line format (21 values after the "UWB" tag, 22 fields total):
//   UWB,t_ms,mask,fps,
//       d0,d1,d2,d3,
//       d0f,d1f,d2f,d3f,
//       xr,yr,xs,ys,sent,
//       std0,std1,std2,std3,spk
//
//   mask     = 4-bit anchor-heard bitmask from this AT+RANGE frame (0..0x0F)
//   fps      = UWB frame rate over the last ~1 s
//   d0..d3   = raw per-anchor distances (m), 0 if anchor not heard
//   d0f..d3f = median-filtered distances (m), NaN if filter not warm or
//              not computed this frame (rate-limited to 10 Hz)
//   xr, yr   = trilateration of raw d's (m), NaN if not all 4 heard
//   xs, ys   = position actually sent to FC (m), NaN if gate rejected
//   sent     = 1 if MAVLink VPE emitted this frame, else 0
//   std0..3  = rolling std-dev of raw distance per anchor (m), NaN until
//              window warm; zeros excluded so partial NLOS doesn't inflate it
//   spk      = 4-bit bitmask of anchors flagged as a spike this frame
//              (|d_raw[i] - running median| > SPIKE_THRESH_M)
// -----------------------------------------------------------------
void send_debug_udp(unsigned long t_ms, uint8_t mask, float fps,
                    const float d_raw[4], const float d_filt[4],
                    const float stds[4], uint8_t spike_mask,
                    float xr, float yr, float xs, float ys, bool sent)
{
#if ENABLE_WIFI_DEBUG
  char buf[384];
  int n = snprintf(buf, sizeof(buf),
                   "UWB,%lu,%u,%.1f,"
                   "%.3f,%.3f,%.3f,%.3f,"
                   "%.3f,%.3f,%.3f,%.3f,"
                   "%.3f,%.3f,%.3f,%.3f,%d,"
                   "%.4f,%.4f,%.4f,%.4f,%u\n",
                   t_ms, (unsigned)mask, fps,
                   d_raw[0], d_raw[1], d_raw[2], d_raw[3],
                   d_filt[0], d_filt[1], d_filt[2], d_filt[3],
                   xr, yr, xs, ys, sent ? 1 : 0,
                   stds[0], stds[1], stds[2], stds[3], (unsigned)spike_mask);
  if (n <= 0) return;
  udp.beginPacket(broadcastIP, udpPort);
  udp.write((const uint8_t *)buf, (size_t)n);
  udp.endPacket();
#else
  (void)t_ms; (void)mask; (void)fps;
  (void)d_raw; (void)d_filt; (void)stds; (void)spike_mask;
  (void)xr; (void)yr; (void)xs; (void)ys; (void)sent;
#endif
}

// -----------------------------------------------------------------
// Broadcasts one cached AT+GET response over UDP so the laptop plotter
// can show it without a USB connection.
//
// Wire format: a single line per key
//   MODINFO,<key>,<value>
//
// The value is the raw module response with CR/LF collapsed to " | "
// so it stays on one CSV line. The plotter splits on the first two
// commas only -- any commas inside <value> are kept verbatim.
// -----------------------------------------------------------------
void broadcast_modinfo_line(const char *key, const String &value)
{
#if ENABLE_WIFI_DEBUG
  String v = value;
  v.replace("\r", "");
  v.replace("\n", " | ");
  v.trim();
  char buf[400];
  int n = snprintf(buf, sizeof(buf), "MODINFO,%s,%s\n", key, v.c_str());
  if (n <= 0) return;
  udp.beginPacket(broadcastIP, udpPort);
  udp.write((const uint8_t *)buf, (size_t)n);
  udp.endPacket();
#else
  (void)key; (void)value;
#endif
}

void broadcast_modinfo_all()
{
  broadcast_modinfo_line("ver", modinfo_ver);
  broadcast_modinfo_line("cfg", modinfo_cfg);
  broadcast_modinfo_line("ant", modinfo_ant);
  broadcast_modinfo_line("cap", modinfo_cap);
  broadcast_modinfo_line("pow", modinfo_pow);
  broadcast_modinfo_line("rpt", modinfo_rpt);
}

// -----------------------------------------------------------------
// Filter the noisy response from an AT+GET* query.
//
// Auto-reporting (AT+RANGE) is running at the module's frame rate, so
// during the 1 s capture window the response string is mostly range-
// data noise with the real GET answer buried inside. Strip every line
// that looks like a range frame and keep the rest.
// -----------------------------------------------------------------
String filter_get_response(const String &raw)
{
  String result = "";
  int from = 0;
  int len = (int)raw.length();
  while (from < len)
  {
    int newline = raw.indexOf('\n', from);
    String line = (newline < 0) ? raw.substring(from) : raw.substring(from, newline);
    line.replace("\r", "");
    line.trim();
    if (line.length() > 0 &&
        line.indexOf("AT+RANGE") < 0 &&
        line.indexOf("range:(") < 0 &&
        line.indexOf("mask:") < 0)
    {
      if (result.length() > 0) result += " | ";
      result += line;
    }
    if (newline < 0) break;
    from = newline + 1;
  }
  if (result.length() == 0) result = "(no response)";
  return result;
}

// -----------------------------------------------------------------
// Range parser: extracts the 4 anchor distances from a LinkTrack
// "AT+RANGE,...,range:(d0,d1,d2,d3,...),..." frame. Distances in cm.
// Writes meters into out[4], with 0.0 for any anchor not heard. Returns
// true if a "range:(...)" block was found and parsed. The caller decides
// whether the frame is usable (e.g. by checking the mask).
// -----------------------------------------------------------------
bool parse_ranges(const String &line, float out[4])
{
  for (int i = 0; i < 4; i++) out[i] = 0.0f;

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
    out[i] = (cm > 0) ? (float)cm / 100.0f : 0.0f;
    if (comma < 0)
      break;
    from = comma + 1;
  }
  return true;
}

// -----------------------------------------------------------------
// Mask parser: extracts the "mask:XX" hex bitfield from an AT+RANGE
// frame -- bit i is set if anchor i was heard this frame. Returns 0
// if no mask was found (treat as "nothing heard").
// -----------------------------------------------------------------
uint8_t parse_mask(const String &line)
{
  int maskIndex = line.indexOf("mask:");
  if (maskIndex < 0) return 0;
  int start = maskIndex + 5;
  int end = line.indexOf(',', start);
  if (end < 0) end = line.length();
  String tok = line.substring(start, end);
  tok.trim();
  return (uint8_t)strtol(tok.c_str(), nullptr, 16);
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
#if ENABLE_WIFI_DEBUG
  // SoftAP for the laptop debug stream. The laptop joins "DroneTracker_UWB"
  // and runs uwb_plot.py to listen on UDP 14550.
  WiFi.softAP(ssid, password);
  udp.begin(udpPort);
  SERIAL_LOG.print("WiFi AP: ");
  SERIAL_LOG.print(ssid);
  SERIAL_LOG.print("   IP: ");
  SERIAL_LOG.println(WiFi.softAPIP());
#else
  SERIAL_LOG.println("WiFi debug DISABLED (ENABLE_WIFI_DEBUG=0). Radio off.");
#endif

  // Hardware connection to the Flight Controller
  SERIAL_FC.begin(115200, SERIAL_8N1, IO_FC_RX, IO_FC_TX);
#endif

  // uwb setup
  pinMode(UWB_RESET, OUTPUT);
  digitalWrite(UWB_RESET, HIGH);

  SERIAL_LOG.begin(115200); // startet die USBSerielle zum PC mit Baud 115200
  // Non-blocking Serial writes: if no USB host is attached (e.g. during
  // flight), Serial.print() would otherwise stall up to tens of ms per
  // call and choke the main loop -- starving MAVLink to the FC and
  // corrupting UWB UART reads. Timeout 0 -> silently discard when no host.
  SERIAL_LOG.setTxTimeoutMs(0);

  // Brief wait for the USB host to enumerate (if one is attached). Bounded
  // so we don't hang forever when running headless.
  unsigned long log_wait_t0 = millis();
  while (!SERIAL_LOG && (millis() - log_wait_t0 < 2000))
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

  // --- Optional: boost TX power to fight CF-chassis attenuation. ----
  // Valid range + units depend on the LinkTrack variant -- check the
  // AT+SETPOW section of your AT-command manual before enabling. Then
  // verify with AT+GETPOW? in the diagnostic dump below.
  // sendData("AT+SETPOW=<value>", 2000, 1);
  // ------------------------------------------------------------------

  sendData("AT+SAVE", 2000, 1);    // Speichert die Konfig im Modul
  sendData("AT+RESTART", 2000, 1); // Startet das Modul mit der neuen Konfig neu

  // --- Diagnostic dump: log module state after the restart so it shows
  //     what was actually applied (not what we asked for). Visible on
  //     the USB console at boot. Useful for ----------------------------
  //       * confirming firmware version when filing a support ticket
  //       * verifying antenna-delay calibration ended up in the module
  //       * checking current TX power
  //       * confirming SETRPT and SETCFG took effect
  // ----------------------------------------------------------------------
  delay(1500); // give the module time to come back from AT+RESTART
  SERIAL_LOG.println(F("--- UWB module state ---"));
#ifdef BUILD_AS_ANCHOR
  sendData("AT+GETVER?", 1000, 1);
  sendData("AT+GETCFG?", 1000, 1);
  sendData("AT+GETANT?", 1000, 1);
  sendData("AT+GETCAP?", 1000, 1);
  sendData("AT+GETPOW?", 1000, 1);
  sendData("AT+GETRPT?", 1000, 1);
#else
  // Capture the responses (filtered to drop the interleaved AT+RANGE
  // auto-report frames) so we can also broadcast them over UDP.
  modinfo_ver = filter_get_response(sendData("AT+GETVER?", 1000, 1));
  modinfo_cfg = filter_get_response(sendData("AT+GETCFG?", 1000, 1));
  modinfo_ant = filter_get_response(sendData("AT+GETANT?", 1000, 1));
  modinfo_cap = filter_get_response(sendData("AT+GETCAP?", 1000, 1));
  modinfo_pow = filter_get_response(sendData("AT+GETPOW?", 1000, 1));
  modinfo_rpt = filter_get_response(sendData("AT+GETRPT?", 1000, 1));
  broadcast_modinfo_all();
  lastModinfo = millis();
#endif
  SERIAL_LOG.println(F("------------------------"));
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

  // Rebroadcast the cached AT+GET module info every modinfoInterval so a
  // laptop that joins the SoftAP mid-flight still receives it.
  if (millis() - lastModinfo >= modinfoInterval)
  {
    lastModinfo = millis();
    broadcast_modinfo_all();
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
        // DEBUG: dump every line we get from the UWB module. Gated by
        // VERBOSE_UWB_LOG -- heavy I/O (10 KB/s) that blocks the loop
        // when no USB host is reading.
#if VERBOSE_UWB_LOG
        SERIAL_LOG.print("[uwb] ");
        SERIAL_LOG.println(response);
#endif

        // Try to parse a range frame. Not every line is one.
        float dist_m[4];
        if (parse_ranges(response, dist_m))
        {
          uint8_t mask = parse_mask(response);

          // "Heard" means the mask bit is set AND the parsed distance is
          // positive (occasionally a frame reports a zero distance for a
          // bit that's nominally set).
          bool heard[4];
          bool all_heard = true;
          for (int i = 0; i < 4; i++)
          {
            heard[i] = ((mask >> i) & 0x01) && (dist_m[i] > 0.0f);
            if (!heard[i]) all_heard = false;
          }

          // --- FPS over a ~1 s sliding window. ---
          fps_counter++;
          unsigned long now_ms = millis();
          if (now_ms - fps_window_start >= 1000)
          {
            unsigned long dt = now_ms - fps_window_start;
            current_fps = (dt > 0) ? ((float)fps_counter * 1000.0f / (float)dt) : 0.0f;
            fps_counter = 0;
            fps_window_start = now_ms;
          }

          // --- Stats ring: store this frame's raw value per anchor (0 if not
          //     heard). Std-dev below excludes zeros. ---
          for (int i = 0; i < 4; i++)
            stats_buf[i][stats_idx] = heard[i] ? dist_m[i] : 0.0f;
          stats_idx = (stats_idx + 1) % STATS_WIN_N;
          if (stats_count < STATS_WIN_N) stats_count++;

          float stds[4] = {NAN, NAN, NAN, NAN};
          if (stats_count >= STATS_WIN_N)
          {
            for (int i = 0; i < 4; i++)
            {
              float sum = 0.0f;
              int n_valid = 0;
              for (int j = 0; j < STATS_WIN_N; j++)
              {
                if (stats_buf[i][j] > 0.0f)
                {
                  sum += stats_buf[i][j];
                  n_valid++;
                }
              }
              if (n_valid < 5) continue; // not enough valid samples
              float mean = sum / (float)n_valid;
              float var = 0.0f;
              for (int j = 0; j < STATS_WIN_N; j++)
              {
                if (stats_buf[i][j] > 0.0f)
                {
                  float d = stats_buf[i][j] - mean;
                  var += d * d;
                }
              }
              stds[i] = sqrtf(var / (float)n_valid);
            }
          }

          // --- Raw single-frame trilateration (unfiltered) for the debug
          //     plot's noise-floor visualisation. Needs all 4 anchors. ---
          float xy_raw_sf[2] = {0.0f, 0.0f};
          bool raw_ok = false;
          if (all_heard) raw_ok = trilaterate_2d(dist_m, xy_raw_sf);
          float xr = raw_ok ? xy_raw_sf[0] : NAN;
          float yr = raw_ok ? xy_raw_sf[1] : NAN;

          // --- Push to the median ring buffer (only when complete). ---
          if (all_heard)
          {
            for (int i = 0; i < 4; i++)
              dist_buf[i][dist_buf_idx] = dist_m[i];
            dist_buf_idx = (dist_buf_idx + 1) % DIST_FILT_N;
            if (dist_buf_count < DIST_FILT_N) dist_buf_count++;
          }

          // --- Per-anchor spike detection: compare current raw to the
          //     running median over the median buffer. Skipped for anchors
          //     not heard this frame. ---
          uint8_t spike_mask = 0;
          if (dist_buf_count >= DIST_FILT_N)
          {
            for (int i = 0; i < 4; i++)
            {
              if (!heard[i]) continue;
              float med = median5(dist_buf[i][0], dist_buf[i][1],
                                  dist_buf[i][2], dist_buf[i][3],
                                  dist_buf[i][4]);
              if (fabsf(dist_m[i] - med) > SPIKE_THRESH_M)
                spike_mask |= (uint8_t)(1 << i);
            }
          }

          // --- Filtered path: rate-limited to 10 Hz, only after buffer warm,
          //     and only on all-heard frames (preserves the original "no VPE
          //     during NLOS bursts" behaviour -- FC's EKF handles the gap). ---
          float dist_filt[4] = {NAN, NAN, NAN, NAN};
          float xs = NAN, ys = NAN;
          bool sent_this_frame = false;

          if (all_heard && dist_buf_count >= DIST_FILT_N &&
              millis() - lastVisionSend >= visionInterval)
          {
            lastVisionSend = millis();

            // Median-filter each anchor's distance.
            for (int i = 0; i < 4; i++)
              dist_filt[i] = median5(dist_buf[i][0], dist_buf[i][1],
                                     dist_buf[i][2], dist_buf[i][3],
                                     dist_buf[i][4]);

            float xy_filt[2];
            if (trilaterate_2d(dist_filt, xy_filt))
            {
              // Velocity gate: reject fixes that imply impossible jumps from
              // the last accepted fix. EKF3 handles smoothing on the FC; we
              // only need to keep gross multipath spikes out of its input.
              bool accept = true;
              float jump = 0.0f;
              if (xy_last_init)
              {
                float dx = xy_filt[0] - xy_last[0];
                float dy = xy_filt[1] - xy_last[1];
                jump = sqrtf(dx * dx + dy * dy);
                if (jump > POS_GATE_M)
                {
                  pos_gate_rejects++;
                  if (pos_gate_rejects < POS_GATE_MAX_REJECT)
                  {
                    accept = false;
                    SERIAL_LOG.printf("[gate] reject jump=%.2f m (rej=%u/%u)\n",
                                      jump, pos_gate_rejects, POS_GATE_MAX_REJECT);
                  }
                  else
                  {
                    SERIAL_LOG.printf("[gate] %u rejects, force-resync to (%.2f, %.2f)\n",
                                      pos_gate_rejects, xy_filt[0], xy_filt[1]);
                  }
                }
              }
              if (accept)
              {
                xy_last[0] = xy_filt[0];
                xy_last[1] = xy_filt[1];
                xy_last_init = true;
                pos_gate_rejects = 0;
                xs = xy_filt[0];
                ys = xy_filt[1];
                sent_this_frame = true;
#if VERBOSE_UWB_LOG
                SERIAL_LOG.printf("d=[%.2f %.2f %.2f %.2f] m  xy=(%.2f, %.2f)\n",
                                  dist_filt[0], dist_filt[1], dist_filt[2], dist_filt[3],
                                  xy_filt[0], xy_filt[1]);
#endif
                send_mavlink_vision_position(xy_filt[0], xy_filt[1]);
              }
            }
            else
            {
              SERIAL_LOG.println("Trilateration failed: anchor geometry singular.");
            }
          }

          // --- Always emit the UDP debug packet, even if the filter wasn't
          //     warm or the gate rejected the frame. The plotter wants to
          //     see all of these states. ---
          send_debug_udp(millis(), mask, current_fps,
                         dist_m, dist_filt, stds, spike_mask,
                         xr, yr, xs, ys, sent_this_frame);
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
//   VISO_DELAY_MS    = 30     // Processing latency UWB sample -> MAVLink, ms.
//                                Tells the EKF when the measurement actually
//                                applies (improves fusion timing). 30 ms is
//                                a safe starting estimate for our pipeline.
//   EK3_VIS_POS_M_NSE = 0.10  // Vision pos measurement noise, m. Start at
//                                your real UWB sigma (~0.05-0.15 m). Higher
//                                = EKF trusts UWB less; lower = trusts more.
//                                This is the main "smoothing" knob now.
//   EK3_POS_I_GATE   = 500    // Position innovation gate, 100*sigma units.
//                                500 = 5 sigma. Tighten to 300 if multipath
//                                spikes still leak through the on-tag gate.
//   GPS_TYPE         = 0      // No GPS indoors
//   ARMING_CHECK     = ...    // May need to disable GPS arming check for indoor
//
// Filtering split:
//   On the tag (this file) we ONLY do (a) median-of-5 on each anchor's range
//   and (b) a 0.5 m velocity gate on the (x, y) fix. Both are outlier rejects,
//   not smoothers. All position smoothing is done by EKF3 on the FC, which
//   has the IMU and produces a much better estimate than anything we could
//   here. If the EKF estimate looks too jumpy, tune EK3_VIS_POS_M_NSE /
//   EK3_POS_I_GATE on the FC -- do NOT add an EMA back into this file.
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