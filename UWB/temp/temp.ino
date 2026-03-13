//  ONLY CHANGE VARIABLES HERE!
// BUILD_AS_ANCHOR nicht auskommentiert -> Anchor, falls auskommentiert dann Tag
#define BUILD_AS_ANCHOR
#define UWB_INDEX "0"

// IMPORTS
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

// VARIABLES
// screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define I2C_SCREEN_SDA 39
#define I2C_SCREEN_SCL 38

// uwb
#define IO_RXD2 18
#define IO_TXD2 17
#define UWB_RESET 16 // IO16
#define UWB_TAG_COUNT 1
#ifdef BUILD_AS_ANCHOR
#define UWB_ROLE "1" // 1 = Anchor
#else
#define UWB_ROLE "0" // 0 = Tag
#endif
#define UWB_TAG_COUNT "1"

// LOG serial
#define SERIAL_LOG Serial
// UWB serial
#define SERIAL_AT mySerial2

// uwb serial object
HardwareSerial SERIAL_AT(2);

// display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// loop variables
String response = "";

// FUNCTIONS
void setup()
{

  // uwb setup
  pinMode(UWB_RESET, OUTPUT);
  digitalWrite(UWB_RESET, HIGH);

  SERIAL_LOG.begin(115200);

  while (!SERIAL_LOG)
  {
    delay(10);
  }
  SERIAL_LOG.println("Starting...");

  SERIAL_AT.begin(115200, SERIAL_8N1, IO_RXD2, IO_TXD2);
  SERIAL_AT.println("AT");

  Wire.begin(I2C_SCREEN_SDA, I2C_SCREEN_SCL);
  delay(1000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    SERIAL_LOG.println("Display connection failed! Check board settings.");
    while (true)
      ;
  }

  printDisplay();

  sendData("AT?", 2000, 1);
  sendData("AT+RESTORE", 5000, 1);
  sendData(config_cmd(), 2000, 1);
  sendData(cap_cmd(), 2000, 1);
  sendData("AT+SETRPT=1", 2000, 1);
  sendData("AT+SAVE", 2000, 1);
  sendData("AT+RESTART", 2000, 1);
}

void loop()
{
  while (SERIAL_LOG.available() > 0)
  {
    SERIAL_AT.write(SERIAL_LOG.read());
    yield();
  }
  while (SERIAL_AT.available() > 0)
  {
    char c = SERIAL_AT.read();

    if (c == '\r')
      continue;
    else if (c == '\n' || c == '\r')
    {
      SERIAL_LOG.println(response);

      response = "";
    }
    else
      response += c;
  }
}

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
  display.println("A" UWB_INDEX);
#else
  display.println("T" UWB_INDEX);
#endif

  display.display();
}

String sendData(String command, const int timeout, boolean debug)
{
  String response = "";
  // command = command + "\r\n";

  SERIAL_LOG.println(command);
  SERIAL_AT.println(command); // send the read character to the SERIAL_LOG

  long int time = millis();

  while ((time + timeout) > millis())
  {
    while (SERIAL_AT.available())
    {

      // The esp has data so display its output to the serial window
      char c = SERIAL_AT.read(); // read the next character.
      response += c;
    }
  }

  if (debug)
  {
    SERIAL_LOG.println(response);
  }

  return response;
}

String config_cmd()
{
  return "AT+SETCFG=" UWB_INDEX "," UWB_ROLE ",1,1";
}

String cap_cmd()
{
  return "AT+SETCAP=" UWB_TAG_COUNT ",10,1";
}