#include "Arduino.h"
#include "pins.h"
#include "Font.h"
#include "EzTime.h"
#include "ArduinoJson.h"
#include "SerialTransfer.h"
#include "SPIFFS.h"
#include "WiFiManager.h"
#include "ESPAsyncWebServer.h"
#include "ESPmDNS.h"
#include "version.h"
#include "ArduinoOTA.h"

#include "motorcontrol.h"

WiFiManager wm;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

SerialTransfer serialTransfer;

void writeBuffer();
void drawChar(char ch, int x, int y);
void drawTime(int hour, int minute);
void drawTimeMove(int hour, int minute, uint16_t timeMs);
void upcomingTime(int &hour12, int &minute);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len);
void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);

uint8_t *psramFirmware = nullptr;
size_t psramFirmwareSize = 0;

void sendFile(String filename);
void sendStatus();

#define WIDTH 8
#define HEIGHT 3
#define CLOCKS (WIDTH * HEIGHT)
#define MODULES CLOCKS / 4

#define CLEAR_DELAY 5000

// In time/cycle modes the hands start moving this many seconds before the
// minute flips, so they ARRIVE on the new time exactly at the flip (move
// duration == lead time). Must be < 60, and for cycle leave room for the :20
// animation to finish first.
#define CYCLE_LEAD 10
#define TIME_LEAD 5

// Holds the buffer for drawing
MotorControl_t buffer[WIDTH][HEIGHT][2];

int calibrateHands0[WIDTH][HEIGHT][2];
int calibrateHands1[WIDTH][HEIGHT][2];

void clearBuffer(MotorControl_t fill = MotorControl_t())
{
  for (int i = 0; i < WIDTH; i++)
  {
    for (int j = 0; j < HEIGHT; j++)
    {
      buffer[i][j][0] = fill;
      buffer[i][j][1] = fill;
    }
  }
}

enum
{
  MODE_CYCLE,
  MODE_TIME,
  MODE_CLEAR,
  MODE_DIAGONAL,
  MODE_CUSTOM,
  MODE_WAVE,
  MODE_ALT_WAVE,
  MODE_ALT_WAVE_DIAG,
  MODE_RADIAL,
  MODE_TEST,
};

volatile int mode = MODE_CYCLE;
String customText = "";

uint8_t moduleMap[8][3][2] = {
    {{0, 0}, {3, 0}, {4, 0}},
    {{0, 1}, {3, 1}, {4, 1}},
    {{0, 2}, {3, 2}, {4, 2}},
    {{0, 3}, {3, 3}, {4, 3}},
    {{1, 0}, {2, 0}, {5, 0}},
    {{1, 1}, {2, 1}, {5, 1}},
    {{1, 2}, {2, 2}, {5, 2}},
    {{1, 3}, {2, 3}, {5, 3}},
};

Timezone myTZ;

bool uploadingFirmware = false;
bool calibrating = false;

void setup()
{
  setCpuFrequencyMhz(240);

  // USB Serial
  Serial.begin(115200);
  Serial.print("Helooo. I am the Master. I am V");
  Serial.print(VERSION_BUILD);
  Serial.print(". I was built on ");
  Serial.print(VERSION_DATE);
  Serial.print(" at ");
  Serial.println(VERSION_TIME);

  if (!psramFound())
  {
    Serial.println("No PSRAM detected!");
  }
  else
  {
    Serial.println("PSRAM available!");
  }

  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Initialize WiFi
  wm.setDarkMode(true);
  wm.setWiFiAutoReconnect(true);
  wm.setConfigPortalBlocking(true);
  wm.autoConnect("ClockClock");

  // Keep the connection alive: persist credentials and let the Arduino
  // core auto-reconnect. A watchdog in loop() forces reconnects if the
  // core gives up (it stops retrying after repeated failures).
  WiFi.persistent(true);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);

  esp_wifi_set_ps(WIFI_PS_NONE);

  ArduinoOTA.setHostname("clockclock");
  ArduinoOTA.begin();

  MDNS.begin("clockclock");
  MDNS.addService("http", "tcp", 80);

  waitForSync();

  // Provide official timezone names
  // https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
  myTZ.setLocation(F("America/Los_Angeles"));
  Serial.print(F("Los Angeles:     "));
  Serial.println(myTZ.dateTime());

  // Wait a little bit to not trigger DDoS protection on server
  // See https://github.com/ropg/ezTime#timezonedropnl
  delay(1000);

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", String()); });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/style.css", "text/css"); });

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/favicon.ico", "image/x-icon"); });

  server.on("/clockclock-cv.js", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/clockclock-cv.js", "application/javascript"); });

  // run handleUpload function when any file is uploaded
  server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request)
            { request->send(200); }, handleUpload);

  // Route to download firmware from PSRAM
  server.on("/firmware", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    if (!psramFirmware || psramFirmwareSize == 0) {
      request->send(404, "text/plain", "No firmware available in PSRAM");
      return;
    }
    
    // Create response with firmware binary data
    AsyncWebServerResponse *response = request->beginResponse(
      "application/octet-stream",
      psramFirmwareSize,
      [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        size_t remaining = psramFirmwareSize - index;
        size_t toSend = (remaining < maxLen) ? remaining : maxLen;
        
        if (toSend > 0 && psramFirmware) {
          memcpy(buffer, psramFirmware + index, toSend);
        }
        
        return toSend;
      }
    );
    
    // Set headers to trigger download
    response->addHeader("Content-Disposition", "attachment; filename=firmware.bin");
    response->addHeader("Content-Length", String(psramFirmwareSize));
    
    request->send(response);
    
    Serial.println("Firmware download requested, size: " + String(psramFirmwareSize) + " bytes"); });

  // Start webserver
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.begin();

  Serial1.setPins(IC1, UART_A);
  Serial1.begin(2000000);

  serialTransfer.begin(Serial1);
}

volatile bool modeChanged = true;
int lastSecond = -1;
int lastDrawnMinute = -1; // minute value the hands are currently showing/targeting
bool wasConnected = true;

void drawTime(int hour, int minute)
{
  drawChar(48 + hour / 10 % 10, 0, 0);
  drawChar(48 + hour % 10, 2, 0);
  drawChar(48 + minute / 10 % 10, 4, 0);
  drawChar(48 + minute % 10, 6, 0);
}

// Draw hour:minute and ship it as a single move that takes timeMs to complete.
void drawTimeMove(int hour, int minute, uint16_t timeMs)
{
  clearBuffer({.position = 135, .time = timeMs, .optimize = true});
  drawTime(hour, minute);
  writeBuffer();
}

// The hour (1-12) and minute that will be showing at the next minute boundary.
void upcomingTime(int &hour12, int &minute)
{
  minute = (myTZ.minute() + 1) % 60;
  if (minute != 0)
  {
    hour12 = myTZ.hourFormat12();
  }
  else
  {
    int h12 = (myTZ.hour() + 1) % 24 % 12;
    hour12 = (h12 == 0) ? 12 : h12;
  }
}

void wave()
{
  for (int i = WIDTH - 1; i >= 0; i--)
  {
    for (int j = 0; j < HEIGHT; j++)
    {
      buffer[i][j][0].position = 135;
      buffer[i][j][1].position = 315;
      buffer[i][j][0].time = 4000;
      buffer[i][j][1].time = 4000;
      buffer[i][j][0].optimize = false;
      buffer[i][j][1].optimize = false;
    }

    writeBuffer();
    delay(400);
  }

  delay(4000);

  for (int i = WIDTH - 1; i >= 0; i--)
  {
    for (int j = 0; j < HEIGHT; j++)
    {
      buffer[i][j][0].position = 135 - (i * 10);
      buffer[i][j][1].position = 315 - (i * 10);
      buffer[i][j][0].speed = 40;
      buffer[i][j][1].speed = 40;
      buffer[i][j][0].keepRunning = true;
      buffer[i][j][1].keepRunning = true;
      buffer[i][j][0].direction = MotorDirection_t::MOTOR_CW;
      buffer[i][j][1].direction = MotorDirection_t::MOTOR_CW;
    }

    writeBuffer();
    delay(600);
  }
  writeBuffer();
}

void altwave()
{
  for (int i = WIDTH - 1; i >= 0; i--)
  {
    for (int j = 0; j < HEIGHT; j++)
    {
      buffer[i][j][0].position = 45;
      buffer[i][j][1].position = 315;
      buffer[i][j][0].time = 4000;
      buffer[i][j][1].time = 4000;
      buffer[i][j][0].optimize = false;
      buffer[i][j][1].optimize = false;
    }

    writeBuffer();
    delay(400);
  }

  delay(4000);

  for (int i = WIDTH - 1; i >= 0; i--)
  {
    for (int j = 0; j < HEIGHT; j++)
    {
      buffer[i][j][0].position = 45 + (i * 10);
      buffer[i][j][1].position = 315 - (i * 10);
      buffer[i][j][0].speed = 40;
      buffer[i][j][1].speed = 40;
      buffer[i][j][0].keepRunning = true;
      buffer[i][j][1].keepRunning = true;
      buffer[i][j][0].direction = MotorDirection_t::MOTOR_CCW;
      buffer[i][j][1].direction = MotorDirection_t::MOTOR_CW;
    }

    writeBuffer();
    delay(400);
  }
  writeBuffer();
}

void altwavediag()
{
  for (int i = WIDTH - 1; i >= 0; i--)
  {
    for (int j = 0; j < HEIGHT; j++)
    {
      buffer[i][j][0].position = 135;
      buffer[i][j][1].position = 315;
      buffer[i][j][0].time = 4000;
      buffer[i][j][1].time = 4000;
      buffer[i][j][0].optimize = false;
      buffer[i][j][1].optimize = false;
    }
    writeBuffer();
    delay(400);
  }

  writeBuffer();
  delay(4000);

  for (int i = WIDTH - 1; i >= 0; i--)
  {
    for (int j = 0; j < HEIGHT; j++)
    {
      buffer[i][j][0].position = 135 + (i * 10);
      buffer[i][j][1].position = 315 - (i * 10);
      buffer[i][j][0].speed = 40;
      buffer[i][j][1].speed = 40;
      buffer[i][j][0].keepRunning = true;
      buffer[i][j][1].keepRunning = true;
      buffer[i][j][0].direction = MotorDirection_t::MOTOR_CCW;
      buffer[i][j][1].direction = MotorDirection_t::MOTOR_CW;
    }

    writeBuffer();
    delay(400);
  }
  writeBuffer();
}

void radialwave()
{
  for (int i = WIDTH - 1; i >= 0; i--)
  {
    for (int j = 0; j < HEIGHT; j++)
    {
      double baseAngle = atan2(j - (HEIGHT / 2.0) + 0.5, i - (WIDTH / 2.0) + 0.5) * (180.0 / PI) + 180;
      double dist = sqrt(pow(i - (WIDTH / 2.0) + 0.5, 2) + pow(j - (HEIGHT / 2.0) + 0.5, 2));
      buffer[i][j][0].position = baseAngle - 22 * dist;
      buffer[i][j][1].position = baseAngle + 22 * dist;
      buffer[i][j][0].time = 4000;
      buffer[i][j][1].time = 4000;
      buffer[i][j][0].optimize = false;
      buffer[i][j][1].optimize = false;
    }
  }

  writeBuffer();

  delay(4000);

  for (int i = WIDTH - 1; i >= 0; i--)
  {
    for (int j = 0; j < HEIGHT; j++)
    {
      buffer[i][j][0].speed = 40;
      buffer[i][j][1].speed = 40;
      buffer[i][j][0].keepRunning = true;
      buffer[i][j][1].keepRunning = true;
      buffer[i][j][0].direction = MotorDirection_t::MOTOR_CCW;
      buffer[i][j][1].direction = MotorDirection_t::MOTOR_CW;
    }
  }

  writeBuffer();
}

void loop()
{

  if (uploadingFirmware)
  {
    Serial.println("Uploading firmware, Clearing Hands");

    clearBuffer({.position = 90, .time = 1000});
    writeBuffer();
    delay(CLEAR_DELAY);

    Serial.println("Uploading firmware");
    sendFile("firmware.bin");
    uploadingFirmware = false;

    delay(5000);

    writeBuffer();

    delay(CLEAR_DELAY);
    modeChanged = true;
  }

  if (!calibrating)
  {
    // Consume the change flag up front. A blocking animation below can run for
    // many seconds; if a new mode arrives during it, the WebSocket task sets
    // this flag again and the next iteration handles it. Clearing at the end
    // instead would discard a change that landed mid-animation, so the new
    // mode would never draw ("nothing happens").
    bool justChanged = modeChanged;
    modeChanged = false;

    switch (mode)
    {
    case MODE_CYCLE:
    {
      int second = myTZ.second();
      int minute = myTZ.minute();

      // Time display: get the hands to LAND on the new minute exactly at the
      // flip. Below the lead window we show the current minute; inside it we
      // move toward the upcoming minute, taking exactly the time left until the
      // boundary so they arrive as it flips.
      if (justChanged)
      {
        drawTimeMove(myTZ.hourFormat12(), minute, CYCLE_LEAD * 1000);
        lastDrawnMinute = minute;
      }
      else if (second < 60 - CYCLE_LEAD)
      {
        if (lastDrawnMinute != minute) // catch-up (e.g. window was missed)
          drawTimeMove(myTZ.hourFormat12(), minute, CYCLE_LEAD * 1000), lastDrawnMinute = minute;
      }
      else // inside the lead window
      {
        int upcomingMinute = (minute + 1) % 60;
        if (lastDrawnMinute != upcomingMinute)
        {
          int upHour, upMin;
          upcomingTime(upHour, upMin);
          drawTimeMove(upHour, upMin, (60 - second) * 1000);
          lastDrawnMinute = upcomingMinute;
        }
      }

      // Play a random animation once at :20 (independent of the time display).
      if (!justChanged && second == 15 && lastSecond != 15 && myTZ.hour() >= 6 && myTZ.hour() < 22)
      {
        int animation = random(0, 5);
        switch (animation)
        {
        case 0:
          wave();
          break;
        case 1:
          altwave();
          break;
        case 2:
          altwavediag();
          break;
        case 3:
          radialwave();
          break;
        case 4:
          for (int i = WIDTH - 1; i >= 0; i--)
          {
            for (int j = 0; j < HEIGHT; j++)
            {
              buffer[i][j][0].speed = 39;
              buffer[i][j][1].speed = 39;
              buffer[i][j][0].keepRunning = true;
              buffer[i][j][1].keepRunning = true;
              buffer[i][j][0].direction = MotorDirection_t::MOTOR_CW;
              buffer[i][j][1].direction = MotorDirection_t::MOTOR_CW;
            }
          }
          writeBuffer();
          break;
        }
        // Leave the hands running the animation; lastDrawnMinute stays at the
        // current minute so nothing redraws the time. The hands keep animating
        // until the lead window (CYCLE_LEAD before the flip), where the move to
        // the upcoming minute takes over and lands on the new time.
      }

      lastSecond = second;
      break;
    }
    case MODE_TIME:
    {
      int second = myTZ.second();
      int minute = myTZ.minute();

      if (justChanged)
      {
        drawTimeMove(myTZ.hourFormat12(), minute, TIME_LEAD * 1000);
        lastDrawnMinute = minute;
      }
      else if (second < 60 - TIME_LEAD)
      {
        if (lastDrawnMinute != minute) // catch-up
          drawTimeMove(myTZ.hourFormat12(), minute, TIME_LEAD * 1000), lastDrawnMinute = minute;
      }
      else
      {
        int upcomingMinute = (minute + 1) % 60;
        if (lastDrawnMinute != upcomingMinute)
        {
          int upHour, upMin;
          upcomingTime(upHour, upMin);
          drawTimeMove(upHour, upMin, (60 - second) * 1000);
          lastDrawnMinute = upcomingMinute;
        }
      }
      break;
    }

    case MODE_CUSTOM:
      if (justChanged)
      {
        clearBuffer({.position = 135, .time = 2000});

        for (int i = 0; i < min((int)customText.length(), 4); i++)
          drawChar(customText.charAt(i), (i * 2) + (4 - customText.length()), 0);

        writeBuffer();
      }
      break;

    case MODE_CLEAR:
      if (justChanged)
      {
        clearBuffer({.position = 90, .time = 1000});

        writeBuffer();
      }
      break;

    case MODE_DIAGONAL:
      if (justChanged)
      {
        clearBuffer({.time = 5000, .optimize = false});

        for (int i = 0; i < WIDTH; i++)
        {
          for (int j = 0; j < HEIGHT; j++)
          {
            buffer[i][j][0].position = 135;
            buffer[i][j][1].position = 315;
          }
        }

        writeBuffer();
      }
      break;

    case MODE_WAVE:
      if (justChanged)
      {
        wave();
      }
      break;

    case MODE_ALT_WAVE:
      if (justChanged)
      {
        altwave();
      }
      break;
    case MODE_ALT_WAVE_DIAG:
      if (justChanged)
      {
        altwavediag();
      }
      break;
    case MODE_RADIAL:
      if (justChanged)
      {
        radialwave();
      }
      break;
    case MODE_TEST:
      if (justChanged)
      {
        clearBuffer({.time = 2000, .optimize = false});

        for (int i = 0; i < WIDTH; i++) {
          for (int j = 0; j < HEIGHT; j++) {
            buffer[i][j][0].position = 135;
            buffer[i][j][1].position = 0;
          }
        }

        writeBuffer();
        
        delay(2500);

        for (int i = 0; i < WIDTH; i++) {
          for (int j = 0; j < HEIGHT; j++) {
            buffer[i][j][0].position = 135;
            buffer[i][j][1].position = 0;
            buffer[i][j][1].speed = 90;
            buffer[i][j][1].keepRunning = true;
          }
        }

        writeBuffer();
      }
      break;
    }
  }

  ArduinoOTA.handle();
  events(); // ezTime event handler to keep time updated

  // WiFi watchdog: the Arduino core auto-reconnect eventually stops trying
  // after repeated failures, leaving the device offline forever. Poll the
  // link and force a reconnect if it has been down for a few seconds.
  static unsigned long lastWiFiCheck = 0;
  static unsigned long downSince = 0;
  bool connected = WiFi.status() == WL_CONNECTED;

  if (millis() - lastWiFiCheck >= 5000)
  {
    lastWiFiCheck = millis();

    if (!connected)
    {
      if (downSince == 0)
      {
        downSince = millis();
      }
      Serial.println("WiFi down, attempting reconnect");
      WiFi.disconnect();
      WiFi.reconnect();

      // If reconnect keeps failing for a long time, fully re-init the radio.
      if (millis() - downSince >= 60000)
      {
        Serial.println("WiFi still down after 60s, reinitializing");
        WiFi.mode(WIFI_OFF);
        delay(100);
        WiFi.mode(WIFI_STA);
        WiFi.begin();
        downSince = millis();
      }
    }
    else
    {
      downSince = 0;
    }
  }

  // Re-register mDNS after WiFi reconnects
  if (connected && !wasConnected)
  {
    Serial.println("WiFi reconnected, re-registering mDNS");
    // The core can re-enable modem sleep on reconnect, which silently
    // breaks mDNS multicast reception. Re-assert sleep-off every time.
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    MDNS.end();
    MDNS.begin("clockclock");
    MDNS.addService("http", "tcp", 80);
  }
  wasConnected = connected;
}

void drawChar(char ch, int x, int y)
{
  auto c = *getCharacter(ch);

  buffer[x][y][0].position = c[0][0][0];
  buffer[x][y][1].position = c[0][0][1];

  buffer[x + 1][y][0].position = c[0][1][0];
  buffer[x + 1][y][1].position = c[0][1][1];

  buffer[x][y + 1][0].position = c[1][0][0];
  buffer[x][y + 1][1].position = c[1][0][1];

  buffer[x + 1][y + 1][0].position = c[1][1][0];
  buffer[x + 1][y + 1][1].position = c[1][1][1];

  buffer[x][y + 2][0].position = c[2][0][0];
  buffer[x][y + 2][1].position = c[2][0][1];

  buffer[x + 1][y + 2][0].position = c[2][1][0];
  buffer[x + 1][y + 2][1].position = c[2][1][1];
}

void writeBuffer()
{
  MotorControl_t sendBuffer[CLOCKS][2];

  // convert the x-y buffer array to a linear array for sending to modules arranged in a Z format
  for (int i = 0; i < MODULES; i++)
  {
    int row = i / (WIDTH / 4);
    int column = (row % 2 == 0) ? (i % (WIDTH / 4)) : ((WIDTH / 4) - 1) - (i % (WIDTH / 4)); // if row is odd invert columns

    for (int j = 0; j < 4; j++)
    {
      sendBuffer[i * 4 + j][0] = buffer[(column * 4) + j][row][0];
      sendBuffer[i * 4 + j][1] = buffer[(column * 4) + j][row][1];
    }
  }

  Serial.print("Sending Buffer: [");
  for (int i = 0; i < 24; i++)
  {
    Serial.print("[" + String(sendBuffer[i][0].position) + ", " + String(sendBuffer[i][1].position) + "], ");
  }
  Serial.println("]");

  for (int module = MODULES - 1; module >= 0; --module)
  {
    uint16_t sendSize = 0;
    uint8_t address = module;

    MotorControl_t moduleBuffer[4][2];
    for (int k = 0; k < 4; k++)
    {
      moduleBuffer[k][0] = sendBuffer[module * 4 + k][0];
      moduleBuffer[k][1] = sendBuffer[module * 4 + k][1];
    }

    sendSize = serialTransfer.txObj(address, sendSize);
    sendSize = serialTransfer.txObj(moduleBuffer, sendSize);
    serialTransfer.sendData(sendSize);
  }

  sendStatus();
}

void sendStatus()
{
  String modeName;
  switch (mode)
  {
  case MODE_CYCLE:
    modeName = "cycle";
    break;
  case MODE_TIME:
    modeName = "time";
    break;
  case MODE_CUSTOM:
    modeName = "custom";
    break;
  case MODE_CLEAR:
    modeName = "clear";
    break;
  case MODE_DIAGONAL:
    modeName = "diagonal";
    break;
  case MODE_WAVE:
    modeName = "wave";
    break;
  case MODE_ALT_WAVE:
    modeName = "altwave";
    break;
  case MODE_ALT_WAVE_DIAG:
    modeName = "altwavediag";
    break;
  case MODE_RADIAL:
    modeName = "radial";
    break;
  case MODE_TEST:
    modeName = "test";
    break;
  }

  // Check if firmware exists in PSRAM
  bool hasFirmware = (psramFirmware != nullptr && psramFirmwareSize > 0);

  // Send status JSON to all WebSocket clients
  String jsonString = "{ \"type\": \"status\", \"mode\": \"" + modeName + "\", \"hasFirmware\": " + (hasFirmware ? "true" : "false") + " }";
  ws.textAll(jsonString);

  // Send buffer to webpage. Each hand is sent as the full motor control so
  // the page can emulate the real stepper motion (trapezoidal moves + spin):
  //   [position, speed, acceleration, direction, time, keepRunning]
  // direction: 0=CW, 1=CCW, 2=shortest. keepRunning: 0/1.
  jsonString = "{ \"type\": \"hands\", \"hands\": [";
  for (int i = 0; i < 8; i++)
  {
    jsonString += "[";
    for (int j = 0; j < 3; j++)
    {
      jsonString += "[";
      for (int h = 0; h < 2; h++)
      {
        const MotorControl_t &m = buffer[i][j][h];
        jsonString += "[" + String(m.position) + "," + String(m.speed) + "," +
                      String(m.acceleration) + "," + String((int)m.direction) + "," +
                      String(m.time) + "," + String(m.keepRunning ? 1 : 0) + "]";
        if (h < 1)
          jsonString += ",";
      }
      jsonString += "]";
      if (j < 2)
        jsonString += ",";
    }
    jsonString += "]";
    if (i < 7)
      jsonString += ",";
  }
  jsonString += "] }";

  ws.textAll(jsonString);
}

void sendFile(String filename)
{
  if (!psramFirmware || psramFirmwareSize == 0)
  {
    Serial.println("No firmware in PSRAM!");
    return;
  }

  uint32_t fileSize = psramFirmwareSize;

  Serial.println("Sending firmware from PSRAM");
  Serial.println("File size: " + String(fileSize) + " bytes");

  // Send "start firmware update" header
  uint16_t headerSize = 0;
  uint8_t address = 201;
  headerSize = serialTransfer.txObj(address, headerSize);
  headerSize = serialTransfer.txObj(fileSize, headerSize);
  serialTransfer.sendData(headerSize);

  delay(250); // wait for modules to forward headers and switch to passthrough mode

  // Split into packets
  uint8_t dataLen = MAX_PACKET_SIZE - 4;
  uint16_t numPackets = fileSize / dataLen; // Reserve bytes for current file index

  if (fileSize % dataLen) // Add an extra transmission if needed
    numPackets++;

  for (uint16_t i = 0; i < numPackets; i++) // Send all data within the file across multiple packets
  {
    uint32_t fileIndex = i * dataLen; // Determine the current file index

    if ((fileIndex + dataLen) > fileSize) // Determine data length for the last packet if file length is not an exact multiple of MAX_PACKET_SIZE-4
      dataLen = fileSize - fileIndex;

    uint8_t sendSize = serialTransfer.txObj(fileIndex);                           // Stuff the current file index
    sendSize = serialTransfer.txObj(psramFirmware[fileIndex], sendSize, dataLen); // Stuff the current file data

    serialTransfer.sendData(sendSize, 1); // Send the current file index and data
    Serial.println("Sending Packet: " + String(i) + " of " + String(numPackets) + " with size: " + String(dataLen) + " bytes");

    if (i % 20 == 0)
      ws.textAll("{ \"type\": \"firmwareUpdate\", \"progress\": " + String((i + 1) * 100 / numPackets) + " }");

    delay(5); // Needed to not overrun RX buffer
  }

  ws.textAll("{ \"type\": \"firmwareUpdate\", \"progress\": 100 }");

  // Signal end of firmware
  address = 202;
  uint8_t sendSize = 0;
  sendSize = serialTransfer.txObj(address, sendSize); // Stuff the current file index

  serialTransfer.sendData(sendSize, 2); // Send the current file index and data

  Serial.println("Firmware upload complete.");
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;

  // Handle the initial JSON message to start the firmware upload
  if (info->index == 0 && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, data);

    if (error)
    {
      Serial.println("Failed to parse JSON");
      return;
    }

    String type = doc["type"];
    if (type == "mode")
    {
      String newmode = doc["mode"];
      Serial.printf("Mode: %s\n", newmode.c_str());

      if (newmode == "cycle")
        mode = MODE_CYCLE;
      else if (newmode == "time")
        mode = MODE_TIME;
      else if (newmode == "custom")
        mode = MODE_CUSTOM;
      else if (newmode == "clear")
        mode = MODE_CLEAR;
      else if (newmode == "diagonal")
        mode = MODE_DIAGONAL;
      else if (newmode == "wave")
        mode = MODE_WAVE;
      else if (newmode == "altwave")
        mode = MODE_ALT_WAVE;
      else if (newmode == "altwavediag")
        mode = MODE_ALT_WAVE_DIAG;
      else if (newmode == "radial")
        mode = MODE_RADIAL;
      else if (newmode == "test")
        mode = MODE_TEST;

      customText = doc["custom"].as<String>();

      modeChanged = true;

      sendStatus();
    }
    else if (type == "firmware")
    {
      Serial.println("Installing firmware on modules");
      uploadingFirmware = true;
    }
    else if (type == "reboot")
    {
      Serial.println("Rebooting modules");
      ESP.restart();
    }
    else if (type == "calibrate")
    {
      int step = doc["step"];

      switch (step)
      {
      case 0:
        Serial.println("Calibrating modules");

        calibrating = true;

        clearBuffer({.time = 1000, .optimize = false});

        for (int i = 0; i < WIDTH; i++)
        {
          for (int j = 0; j < HEIGHT; j++)
          {
            buffer[i][j][0].position = 0;
            buffer[i][j][1].position = 90;
          }
        }
        writeBuffer();
        break;
      case 1:
      {
        // Accept either a JSON array or a JSON string containing the array
        DynamicJsonDocument handsDoc(4096);
        JsonArray hands;

        if (doc["hands"].is<JsonArray>())
        {
          hands = doc["hands"].as<JsonArray>();
        }
        else
        {
          String handsStr = doc["hands"].as<String>();
          DeserializationError err = deserializeJson(handsDoc, handsStr);
          if (err)
          {
            Serial.println("Failed to parse hands JSON");
            return;
          }
          hands = handsDoc.as<JsonArray>();
        }

        int i = 0;
        for (JsonArray x : hands)
        {
          int j = 0;
          for (JsonArray y : x)
          {
            calibrateHands0[i][j][0] = y[0];
            calibrateHands0[i][j][1] = y[1];
            j++;
          }
          i++;
        }

        for (int x = 0; x < WIDTH; x++)
        {
          for (int y = 0; y < HEIGHT; y++)
          {
            buffer[x][y][0].position += 180;
            buffer[x][y][0].time = 1000;
            buffer[x][y][1].time = 1000;
            buffer[x][y][0].optimize = false;
            buffer[x][y][1].optimize = false;
          }
        }

        writeBuffer();
        break;
      }
      case 2:
      {
        // Accept either a JSON array or a JSON string containing the array
        DynamicJsonDocument handsDoc(4096);
        JsonArray hands;

        if (doc["hands"].is<JsonArray>())
        {
          hands = doc["hands"].as<JsonArray>();
        }
        else
        {
          String handsStr = doc["hands"].as<String>();
          DeserializationError err = deserializeJson(handsDoc, handsStr);
          if (err)
          {
            Serial.println("Failed to parse hands JSON");
            return;
          }
          hands = handsDoc.as<JsonArray>();
        }

        int i = 0;
        for (JsonArray x : hands)
        {
          int j = 0;
          for (JsonArray y : x)
          {
            calibrateHands1[i][j][0] = y[0];
            calibrateHands1[i][j][1] = y[1];
            j++;
          }
          i++;
        }

        int currentHands[WIDTH][HEIGHT][2];
        for (int x = 0; x < WIDTH; x++)
        {
          for (int y = 0; y < HEIGHT; y++)
          {
            currentHands[x][y][0] = calibrateHands1[x][y][0];
            currentHands[x][y][1] = calibrateHands1[x][y][1];
          }
        }

        for (int x = 0; x < WIDTH; x++)
        {
          for (int y = 0; y < HEIGHT; y++)
          {
            // Determine which camera hand in step 1 is the physical hand that moved (+180°).
            // The camera may reorder hands between captures, so we check all 4 pairings
            // and find which step1 hand is ~180° away from any step0 hand.

            // Angular difference helper (accounts for wraparound)
            auto angDiff = [](int a, int b) -> int {
              int d = abs(a - b);
              return d > 180 ? 360 - d : d;
            };

            // How close is each step1 hand to being 180° from any step0 hand?
            int err0 = min(abs(angDiff(currentHands[x][y][0], calibrateHands0[x][y][0]) - 180),
                           abs(angDiff(currentHands[x][y][0], calibrateHands0[x][y][1]) - 180));
            int err1 = min(abs(angDiff(currentHands[x][y][1], calibrateHands0[x][y][0]) - 180),
                           abs(angDiff(currentHands[x][y][1], calibrateHands0[x][y][1]) - 180));

            if (err1 < err0)
            {
              // Camera hand 1 is closer to a 180° change — it's the physical hand that moved (hand 0)
              // Swap so currentHands[0] = moved hand, currentHands[1] = stayed hand
              int temp0 = currentHands[x][y][0];
              currentHands[x][y][0] = currentHands[x][y][1];
              currentHands[x][y][1] = temp0;
            }
          }
        }

        for (int x = 0; x < WIDTH; x++)
        {
          for (int y = 0; y < HEIGHT; y++)
          {
            // offset = commanded_position - detected_angle
            // corrected_command = target + offset
            // target = 180° in camera coords (0°=12 o'clock) = 90° in motor coords = straight down
            buffer[x][y][0].position = buffer[x][y][0].position + 180 - currentHands[x][y][0];
            buffer[x][y][1].position = buffer[x][y][1].position + 180 - currentHands[x][y][1];
            buffer[x][y][0].time = 1000;
            buffer[x][y][1].time = 1000;
            buffer[x][y][0].optimize = false;
            buffer[x][y][1].optimize = false;
          }
        }

        writeBuffer();

        delay(1200);

        // Signal end of firmware
        uint8_t address = 220;
        uint8_t sendSize = 0;
        sendSize = serialTransfer.txObj(address, sendSize); // Stuff the current file index

        serialTransfer.sendData(sendSize);

        delay(500);

        calibrating = false;
        modeChanged = true;
        break;
      }
      case 99:
        calibrating = false;
        modeChanged = true;
        break;
      }
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    sendStatus();
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

// handles uploads
void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
  static bool installFirmware = false;

  if (!index) // first chunk
  {
    Serial.println("Upload Start: " + filename);
    // Read install header
    const AsyncWebHeader *h = request->getHeader("install");
    installFirmware = (h && h->value() == "true");

    // Allocate PSRAM for the firmware file
    if (psramFirmware)
      free(psramFirmware);                        // free previous buffer
    psramFirmwareSize = request->contentLength(); // total file size
    psramFirmware = (uint8_t *)heap_caps_malloc(psramFirmwareSize, MALLOC_CAP_SPIRAM);
    if (!psramFirmware)
    {
      Serial.println("Failed to allocate PSRAM buffer!");
      return;
    }

    Serial.println("Allocated PSRAM buffer: " + String(psramFirmwareSize) + " bytes, Install: " + installFirmware);
  }

  if (len && psramFirmware)
  {
    memcpy(psramFirmware + index, data, len);
    Serial.println("Writing chunk: index=" + String(index) + " len=" + String(len));
  }

  if (final)
  {
    Serial.println("Upload Complete: " + filename + ", size=" + String(psramFirmwareSize));
    request->redirect("/");

    sendStatus();

    if (installFirmware)
    {
      Serial.println("Installing firmware from PSRAM");
      uploadingFirmware = true;
    }
  }
}