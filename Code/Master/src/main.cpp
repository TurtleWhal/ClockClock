#include <Arduino.h>
// #include "ClockSerial.h" // includes Data.h
#include "pins.h"
#include "Font.h"

#include "EzTime.h"
#include "ArduinoJson.h"

#include "SerialTransfer.h"

#include "SPIFFS.h"
#include "WiFiManager.h"
#include "ESPAsyncWebServer.h"
#include "ESPmDNS.h"
WiFiManager wm;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ClockSerial clockSerial;
SerialTransfer serialTransfer;

// void handleMessage(Data *data);
void writeBuffer();
void drawChar(uint8_t num, int x, int y);
void drawTime();
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len);

// Holds the buffer for drawing
uint16_t buffer[8][3][2];

#define DEG_TO_STEPS 2

#define MODE_TIME 0
#define MODE_CUSTOM 1
#define MODE_CLEAR 2

int mode = MODE_TIME;

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
  // Serial.setTxTimeoutMs(0);
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
  delay(1000);

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
  // clockSerial.onRecieve(handleMessage);
  // clockSerial.begin(IC1, UART_A, true);

  Serial1.setPins(IC1, UART_A);
  Serial1.begin(2000000);

  serialTransfer.begin(Serial1);
  // serialTransfer.begin(Serial1);
}

bool modeChanged = true;
int lastMinute = -1;

void loop()
{
  switch (mode)
  {
  case MODE_TIME:
    if (lastMinute != myTZ.minute() || modeChanged)
    {
      drawTime();
      writeBuffer();
      lastMinute = myTZ.minute();
    }
    break;

  case MODE_CUSTOM:
    // TODO
    break;

  case MODE_CLEAR:
    if (modeChanged)
    {
      for (int i = 0; i < 8; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          buffer[i][j][0] = 90;
          buffer[i][j][1] = 90;
        }
      }

      writeBuffer();
    }
    break;
  }

  // Serial.println(WiFi.isConnected());
  // Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  // Serial.printf("Max allocatable block: %d\n", ESP.getMaxAllocHeap());

  // modeChanged = false;
  modeChanged = true;
  delay(5000);
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
}

File firmwareFile;
size_t totalReceived = 0;
size_t firmwareSize = 0;

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
    if (type == "firmware")
    {
      firmwareSize = doc["size"]; // Get the firmware size from JSON
      totalReceived = 0;

      // Open the firmware file for writing
      firmwareFile = SPIFFS.open("/firmware.bin", FILE_WRITE);
      if (!firmwareFile)
      {
        Serial.println("Failed to open file for writing");
        return;
      }

      Serial.println("Firmware upload initiated");
    }
    else if (type == "mode")
    {
      String newmode = doc["mode"];
      Serial.printf("Mode: %s\n", newmode.c_str());

      if (newmode == "time")
        mode = MODE_TIME;
      else if (newmode == "custom")
        mode = MODE_CUSTOM;
      else if (newmode == "clear")
        mode = MODE_CLEAR;

      modeChanged = true;
    }
  }

  // Handle the binary data (firmware) chunks
  if (info->opcode == WS_BINARY && firmwareFile)
  {
    // Write the received binary chunk to the file
    firmwareFile.write(data, len);
    totalReceived += len;

    Serial.printf("Received %d/%d bytes\n", totalReceived, firmwareSize);
  }

  // Close the file and complete the upload when the transmission is finished
  if (info->final && firmwareFile)
  {
    firmwareFile.close();
    Serial.println("Firmware upload completed");

    if (totalReceived == firmwareSize)
    {
      Serial.println("Firmware file written successfully.");
    }
    else
    {
      Serial.println("Warning: Firmware size mismatch!");
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

// void handleMessage(Data *data)
// {
//   // Serial.println("Message received");
// }

void writeBuffer()
{
  uint16_t sendSize = 0;

  sendSize = serialTransfer.txObj(buffer, sendSize);
  // sendSize = serialTransfer.txObj(0x0123456789ABCDEF, sendSize);

  Serial.println("sendSize: " + String(sendSize));
  serialTransfer.sendData(sendSize);
  Serial.println("sent");

  Serial.print(buffer[6][0][0]);
  Serial.print(" ");
  Serial.print(buffer[6][0][1]);
  Serial.print(", ");
  Serial.print(buffer[7][0][0]);
  Serial.print(" ");
  Serial.print(buffer[7][0][1]);
  Serial.println();

  // send to all modules
  // for (int j = 0; j < 3; j++)
  // {
  //   for (int i = 0; i < 8; i++)
  //   {
  // clockSerial.send(new Data(moduleMap[i][j][0], moduleMap[i][j][1], 0, buffer[i][j][0] * DEG_TO_STEPS));
  // clockSerial.send(new Data(moduleMap[i][j][0], moduleMap[i][j][1], 1, buffer[i][j][1] * DEG_TO_STEPS));
  // Data *bottom = new Data(moduleMap[i][j][0], moduleMap[i][j][1], 0, buffer[i][j][0] * DEG_TO_STEPS);
  // Data *top = new Data(moduleMap[i][j][0], moduleMap[i][j][1], 1, buffer[i][j][1] * DEG_TO_STEPS);
  // clockSerial.send(bottom);
  // clockSerial.send(top);
  // free(bottom);
  // free(top);
  //   }
  // }

  // send to webpage
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
