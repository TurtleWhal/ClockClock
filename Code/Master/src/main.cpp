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
void drawChar(uint8_t num, int x, int y);
void drawTime();
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len);
void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);

void sendFile(String filename);
void sendStatus();

#define WIDTH 8
#define HEIGHT 3
#define CLOCKS WIDTH *HEIGHT
#define MODULES CLOCKS / 4

#define CLEAR_DELAY 5000

// Holds the buffer for drawing
// int buffer[WIDTH][HEIGHT][2];
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

enum {
  MODE_TIME,
  MODE_CLEAR,
  MODE_DIAGONAL,
  MODE_CUSTOM,
  MODE_WAVE,
  MODE_ALT_WAVE,
};

int mode = MODE_TIME;
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

  // // run handleUpload function when any file is uploaded
  // server.on("/upload", HTTP_GET, [](AsyncWebServerRequest *request)
  //           { request->send(200, "text/html", listFiles(true)); });

  // Start webserver
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.begin();

  Serial1.setPins(IC1, UART_A);
  Serial1.begin(2000000);

  serialTransfer.begin(Serial1);
}

bool modeChanged = true;
int lastMinute = -1;

void loop()
{

  if (uploadingFirmware)
  {
    Serial.println("Uploading firmware, Clearing Hands");

    clearBuffer({.position = 90, .time = 1000});
    // for (int i = 0; i < WIDTH; i++)
    // {
    //   for (int j = 0; j < HEIGHT; j++)
    //   {
    //     buffer[i][j][0].position = 90;
    //     buffer[i][j][1].position = 90;
    //   }
    // }
    // TODO: was speed false and optimize false, also make writebuffer calculate the amount of time to get to position
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
  case MODE_TIME:
    if (lastMinute != myTZ.minute() || modeChanged)
    {
      clearBuffer({.position = 135, .time = 5000});

      int hour = myTZ.hourFormat12();
      int minute = myTZ.minute();
      int second = myTZ.second();

      drawChar(hour / 10 % 10, 0, 0);
      drawChar(hour % 10, 2, 0);
      drawChar(minute / 10 % 10, 4, 0);
      drawChar(minute % 10, 6, 0);

      writeBuffer();
      lastMinute = myTZ.minute();
    }
    break;

  case MODE_CUSTOM:
    if (modeChanged)
    {
      clearBuffer({.position = 135, .time = 5000});

      // for (int i = 0; i < WIDTH; i++)
      // {
      //   for (int j = 0; j < HEIGHT; j++)
      //   {
      //     buffer[i][j][0].position = 135;
      //     buffer[i][j][1].position = 135;
      //   }
      // }

      for (int i = 0; i < min((int)customText.length(), 4); i++)
      {
        drawChar(CHAR_TO_FONT(customText.charAt(i)), (i * 2) + (4 - customText.length()), 0);
      }
      writeBuffer();
    }
    break;

  case MODE_CLEAR:
    if (modeChanged)
    {
      clearBuffer({.position = 90, .time = 1000});

      // for (int i = 0; i < WIDTH; i++)
      // {
      //   for (int j = 0; j < HEIGHT; j++)
      //   {
      //     buffer[i][j][0].position = 90;
      //     buffer[i][j][1].position = 90;
      //   }
      // }

      writeBuffer();
    }
    break;

  case MODE_DIAGONAL:
    if (modeChanged)
    {
      clearBuffer({.time = 5000});

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
      clearBuffer();

      for (int i = 0; i < WIDTH; i++)
      {
        for (int j = 0; j < HEIGHT; j++)
        {
          buffer[i][j][0].position = 135;
          buffer[i][j][1].position = 315;
          buffer[i][j][0].direction = MOTOR_CW;
          buffer[i][j][1].direction = MOTOR_CW;
        }
      }
      
      writeBuffer();
      delay(CLEAR_DELAY);
      
      for (int i = 0; i < WIDTH; i++)
      {
        for (int j = 0; j < HEIGHT; j++)
        {
          buffer[i][j][0].position = 135 - (i * 10);
          buffer[i][j][1].position = 315 - (i * 10);
          buffer[i][j][0].speed = 100;
          buffer[i][j][1].speed = 100;
          buffer[i][j][0].keepRunning = true;
          buffer[i][j][1].keepRunning = true;
          buffer[i][j][0].direction = MOTOR_CW;
          buffer[i][j][1].direction = MOTOR_CW;
        }
        
        writeBuffer();
        delay(600);
      }
      writeBuffer();
    }
    break;
    
    case MODE_ALT_WAVE:
    if (modeChanged)
    {
      clearBuffer();
      
      for (int i = 0; i < WIDTH; i++)
      {
        for (int j = 0; j < HEIGHT; j++)
        {
          buffer[i][j][0].position = 90;
          buffer[i][j][1].position = 270;
          buffer[i][j][0].direction = MOTOR_CCW;
          buffer[i][j][1].direction = MOTOR_CW;
        }
      }
      
      // writeBuffer();
      // delay(CLEAR_DELAY);
      
      for (int i = 0; i < WIDTH; i++)
      {
        for (int j = 0; j < HEIGHT; j++)
        {
          buffer[i][j][0].position = 90 + (i * 10);
          buffer[i][j][1].position = 270 - (i * 10);
          buffer[i][j][0].speed = 100;
          buffer[i][j][1].speed = 100;
          buffer[i][j][0].keepRunning = true;
          buffer[i][j][1].keepRunning = true;
          buffer[i][j][0].direction = MOTOR_CCW;
          buffer[i][j][1].direction = MOTOR_CW;
        }

        writeBuffer();
        delay(400);
      }
      writeBuffer();
    }
    break;
  }

  // Serial.println(WiFi.isConnected());
  // Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  // Serial.printf("Max allocatable block: %d\n", ESP.getMaxAllocHeap());

  modeChanged = false; // update at least every 5 seconds

  // modeChanged = true;
  // delay(5000);
  // vTaskDelay(1000 / portTICK_PERIOD_MS);

  ArduinoOTA.handle();

  delay(100);
}

void writeBuffer()
{
  Serial.println("Sending Buffer");

  // if (!uploadingFirmware)
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

  uint16_t sendSize = 0;
  uint8_t address = 0;
  sendSize = serialTransfer.txObj(address, sendSize);
  sendSize = serialTransfer.txObj(sendBuffer, sendSize);
  serialTransfer.sendData(sendSize);

  sendStatus();
}

void sendStatus()
{
  String modeName;
  switch (mode)
  {
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
  }
  // Send the constructed JSON string over WebSocket to all connected clients
  String jsonString = "{ \"type\": \"status\", \"mode\": \"" + modeName + "\", \"hasFirmware\": " + (SPIFFS.exists("/firmware.bin") ? "true" : "false") + " }";
  ws.textAll(jsonString);

  // send buffer to webpage
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

void sendFile(String filename)
{
  // Send file to modules
  Serial.println("Opening file: " + filename);
  File firmware = SPIFFS.open("/" + filename, "r");
  Serial.println("File size: " + String(firmware.size()) + " bytes");
  char *data = new char[firmware.size()];
  firmware.readBytes(data, firmware.size());

  uint32_t fileSize = firmware.size();

  uint16_t headerSize = 0;

  uint8_t address = 201;
  headerSize = serialTransfer.txObj(address, headerSize);
  headerSize = serialTransfer.txObj(fileSize, headerSize);

  serialTransfer.sendData(headerSize);

  delay(250); // wait for modules to forward headers and switch to passthrough mode

  uint8_t dataLen = MAX_PACKET_SIZE - 4;
  uint16_t numPackets = fileSize / dataLen; // Reserve two bytes for current file index

  if (fileSize % dataLen) // Add an extra transmission if needed
    numPackets++;

  for (uint16_t i = 0; i < numPackets; i++) // Send all data within the file across multiple packets
  {
    uint32_t fileIndex = i * dataLen; // Determine the current file index

    if ((fileIndex + dataLen) > fileSize) // Determine data length for the last packet if file length is not an exact multiple of MAX_PACKET_SIZE-2
      dataLen = fileSize - fileIndex;

    uint8_t sendSize = serialTransfer.txObj(fileIndex);                  // Stuff the current file index
    sendSize = serialTransfer.txObj(data[fileIndex], sendSize, dataLen); // Stuff the current file data

    serialTransfer.sendData(sendSize, 1); // Send the current file index and data
    Serial.println("Sending Packet: " + String(i) + " of " + String(numPackets) + " with size: " + String(dataLen) + " bytes");

    if (i % 20 == 0)
      ws.textAll("{ \"type\": \"firmwareUpdate\", \"progress\": " + String((i + 1) * 100 / numPackets) + " }");

    delay(5); // Needed to not overrun RX buffer
  }

  ws.textAll("{ \"type\": \"firmwareUpdate\", \"progress\": 100 }");

  address = 202;
  uint8_t sendSize = 0;
  sendSize = serialTransfer.txObj(address, sendSize); // Stuff the current file index

  serialTransfer.sendData(sendSize, 2); // Send the current file index and data
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

      if (newmode == "time")
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

      customText = doc["custom"].as<String>();
      customText.toUpperCase();

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
  // filename = "firmware.bin";
  String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();

  bool installFirmware = false;

  if (!index)
  {
    Serial.println(logmessage);

    logmessage = "Upload Start: " + String(filename);
    // open the file on first call and store the file handle in the request object
    request->_tempFile = SPIFFS.open("/" + filename, "w");
    request->getHeader("install")->toString().equals("true") ? installFirmware = true : installFirmware = false;
    Serial.println(logmessage + ", Install: " + installFirmware);
  }

  if (len)
  {
    // stream the incoming chunk to the opened file
    request->_tempFile.write(data, len);
    logmessage = "Writing file: " + String(filename) + " index=" + String(index) + " len=" + String(len);
    Serial.println(logmessage);
  }

  if (final)
  {
    logmessage = "Upload Complete: " + String(filename) + ",size: " + String(index + len);
    // close the file handle as the upload is now done
    request->_tempFile.close();
    Serial.println(logmessage);
    request->redirect("/");

    sendStatus();

    if (installFirmware)
    {
      // sendFile(filename);
      uploadingFirmware = true;
    }
  }
}