#include <Arduino.h>
#include "ClockSerial.h" // includes Data.h
#include "pins.h"
#include "Font.h"

#include "EzTime.h"

#include "SPIFFS.h"
#include "WiFiManager.h"
#include "ESPAsyncWebServer.h"
#include "ESPmDNS.h"
WiFiManager wm;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

ClockSerial clockSerial;

void handleMessage(Data *data);
void writeBuffer();
void drawChar(uint8_t num, int x, int y);
void drawTime();
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len);

// Holds the buffer for drawing
uint16_t buffer[8][3][2];

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

void setup()
{
  // USB Serial
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);
  Serial.println("Helooo");

  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Initialize WiFi
  wm.setDarkMode(true);
  wm.autoConnect("ClockClock");
  MDNS.begin("clockclock");

  waitForSync();

  // Provide official timezone names
  // https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
  myTZ.setLocation(F("America/Los_Angeles"));
  Serial.print(F("Los Angeles:     "));
  Serial.println(myTZ.dateTime());

  // Wait a little bit to not trigger DDoS protection on server
  // See https://github.com/ropg/ezTime#timezonedropnl
  delay(5000);

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", String()); });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/style.css", "text/css"); });

  // Start webserver
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.begin();

  // start clock serial communication
  clockSerial.onRecieve(handleMessage);
  clockSerial.begin(IC1, UART_A);
}

void loop()
{
  drawTime();

  // Manually construct a JSON string
  String jsonString = "{ \"buffer\": [";

  for (int i = 0; i < 8; i++)
  {
    jsonString += "[";
    for (int j = 0; j < 3; j++)
    {
      jsonString += "[";
      jsonString += String(buffer[i][j][0]) + "," + String(buffer[i][j][1]);
      jsonString += "]";
      if (j < 2)
      {
        jsonString += ",";
      }
    }
    jsonString += "]";
    if (i < 7)
    {
      jsonString += ",";
    }
  }

  jsonString += "] }";

  // Send the constructed JSON string over WebSocket to all connected clients
  ws.textAll(jsonString);

  // Delay for 1 minute (60000 milliseconds)
  delay(1000);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    if (strcmp((char *)data, "toggle") == 0)
    {
      // Handle WebSocket message here if necessary
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

void handleMessage(Data *data)
{
  Serial.println("Message received");
}

void writeBuffer()
{
  for (int i = 0; i < 8; i++)
    for (int j = 0; j < 3; j++)
    {
      clockSerial.send(new Data(moduleMap[i][j][0], moduleMap[i][j][1], 0, buffer[i][j][0]));
      clockSerial.send(new Data(moduleMap[i][j][0], moduleMap[i][j][1], 1, buffer[i][j][1]));
    }
}

void drawChar(uint8_t num, int x, int y)
{
  auto c = font[num];

  buffer[x][y][0] = c[0][0][0];
  buffer[x][y][1] = c[0][0][1];

  buffer[x + 1][y][0] = c[0][1][0];
  buffer[x + 1][y][1] = c[0][1][1];

  buffer[x][y + 1][0] = c[1][0][0];
  buffer[x][y + 1][1] = c[1][0][1];

  buffer[x + 1][y + 1][0] = c[1][1][0];
  buffer[x + 1][y + 1][1] = c[1][1][1];

  buffer[x][y + 2][0] = c[2][0][0];
  buffer[x][y + 2][1] = c[2][0][1];

  buffer[x + 1][y + 2][0] = c[2][1][0];
  buffer[x + 1][y + 2][1] = c[2][1][1];
}

void drawTime()
{
  int hour = myTZ.hourFormat12();
  int minute = myTZ.minute();
  int second = myTZ.second();

  drawChar(hour / 10 % 10, 0, 0);
  drawChar(hour % 10, 2, 0);
  drawChar(minute / 10 % 10, 4, 0);
  drawChar(minute % 10, 6, 0);
}
