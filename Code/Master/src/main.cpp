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
void drawTime();
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

// Holds the buffer for drawing
MotorControl_t buffer[WIDTH][HEIGHT][2];

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
};

int mode = MODE_CYCLE;
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

void setup()
{
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
  wm.autoConnect("ClockClock");

  ArduinoOTA.setHostname("clockclock");
  ArduinoOTA.begin();

  // MDNS.begin("clockclock");

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

bool modeChanged = true;
int lastSecond = -1;
int lastMinute = -1;

void drawTime()
{
  int hour = myTZ.hourFormat12();
  int minute = myTZ.minute();
  int second = myTZ.second();

  drawChar(48 + hour / 10 % 10, 0, 0);
  drawChar(48 + hour % 10, 2, 0);
  drawChar(48 + minute / 10 % 10, 4, 0);
  drawChar(48 + minute % 10, 6, 0);
}

void wave()
{
  for (int i = WIDTH; i >= 0; i--)
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

  for (int i = WIDTH; i >= 0; i--)
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
  for (int i = WIDTH; i >= 0; i--)
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

  for (int i = WIDTH; i >= 0; i--)
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
  for (int i = WIDTH; i >= 0; i--)
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

  for (int i = WIDTH; i >= 0; i--)
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

  switch (mode)
  {
  case MODE_CYCLE:
    if (lastSecond != myTZ.second() || modeChanged)
    {
      if (lastMinute != myTZ.minute() || modeChanged)
      {
        clearBuffer({.position = 135, .time = 10000});

        drawTime();

        writeBuffer();

        lastMinute = myTZ.minute();
      }
      else if (lastSecond == 20)
      {
        // randomly select an animation
        int animation = random(0, 3);
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
        }
      }

      lastSecond = myTZ.second();
    }
    break;
  case MODE_TIME:
    if (lastMinute != myTZ.minute() || modeChanged)
    {
      clearBuffer({.position = 135, .time = 5000});

      drawTime();

      writeBuffer();
      lastMinute = myTZ.minute();
    }
    break;

  case MODE_CUSTOM:
    if (modeChanged)
    {
      clearBuffer({.position = 135, .time = 2000});

      for (int i = 0; i < min((int)customText.length(), 4); i++)
        drawChar(customText.charAt(i), (i * 2) + (4 - customText.length()), 0);

      writeBuffer();
    }
    break;

  case MODE_CLEAR:
    if (modeChanged)
    {
      clearBuffer({.position = 90, .time = 1000});

      writeBuffer();
    }
    break;

  case MODE_DIAGONAL:
    if (modeChanged)
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
    if (modeChanged)
    {
      wave();
    }
    break;

  case MODE_ALT_WAVE:
    if (modeChanged)
    {
      altwave();
    }
    break;
  case MODE_ALT_WAVE_DIAG:
    if (modeChanged)
    {
      altwavediag();
    }
    break;
  }

  modeChanged = false;

  ArduinoOTA.handle();
  events(); // ezTime event handler to keep time updated

  delay(100);
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
  }

  // Check if firmware exists in PSRAM
  bool hasFirmware = (psramFirmware != nullptr && psramFirmwareSize > 0);

  // Send status JSON to all WebSocket clients
  String jsonString = "{ \"type\": \"status\", \"mode\": \"" + modeName + "\", \"hasFirmware\": " + (hasFirmware ? "true" : "false") + " }";
  ws.textAll(jsonString);

  // Send buffer positions to webpage
  jsonString = "{ \"type\": \"hands\", \"hands\": [";
  for (int i = 0; i < 8; i++)
  {
    jsonString += "[";
    for (int j = 0; j < 3; j++)
    {
      jsonString += "[";
      jsonString += String(buffer[i][j][0].position) + "," + String(buffer[i][j][1].position);
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

      customText = doc["custom"].as<String>();

      modeChanged = true;

      sendStatus();
    }
    else if (type == "firmware")
    {
      Serial.println("Installing firmware on modules");
      uploadingFirmware = true;
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