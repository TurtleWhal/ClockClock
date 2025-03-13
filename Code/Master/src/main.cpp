#include <Arduino.h>
#include "ClockSerial.h" // includes Data.h
#include "pins.h"
#include "Font.h"

ClockSerial clockSerial;

void handleMessage(Data *data);
void writeBuffer();
void drawChar(uint8_t num, int x, int y);
void drawTime();

uint16_t buffer[8][3][2];

uint8_t moduleMap[8][3][2] = {
    {{0, 0}, {2, 0}, {4, 0}},
    {{0, 1}, {2, 1}, {4, 1}},
    {{0, 2}, {2, 2}, {4, 2}},
    {{0, 3}, {2, 3}, {4, 3}},
    {{1, 0}, {3, 0}, {5, 0}},
    {{1, 1}, {3, 1}, {5, 1}},
    {{1, 2}, {3, 2}, {5, 2}},
    {{1, 3}, {3, 3}, {5, 3}},
};

void setup()
{
  // Serial
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);
  Serial.println("Helooo");

  delay(5000);

  // put your setup code here, to run once:
  clockSerial.onRecieve(handleMessage);
  clockSerial.begin(IC1, UART_A);

  int i = 0;
  while (true)
  {
    clockSerial.send(new Data(0, 0, 0, i));
    delay(200);
    clockSerial.send(new Data(0, 1, 0, i));
    delay(200);
    clockSerial.send(new Data(0, 2, 0, i));
    delay(200);
    clockSerial.send(new Data(0, 3, 0, i));
    delay(200);
    i += 10;
  }
}

void loop()
{
  drawTime();
  delay(60000);
}

void handleMessage(Data *data)
{
  Serial.println("Message received");
  // int address = data.getAddress();

  // if (address > 0)
  // {

  //   // clockSerial.send(data);
  // }
  // else
  // {
  //   // OMG A MESSAGE JUST FOR ME!!!
  // }
}

void writeBuffer()
{
  for (int i = 0; i < 8; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      clockSerial.send(new Data(moduleMap[i][j][0], moduleMap[i][j][1], 0, buffer[i][j][0]));
      clockSerial.send(new Data(moduleMap[i][j][0], moduleMap[i][j][1], 1, buffer[i][j][1]));
    }
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

  writeBuffer();
}

void drawTime() {
  long time = millis();

  int seconds = time / 1000 % 60;
  int minutes = seconds / 60 % 60;
  int hours = minutes / 60 % 24;

  drawChar(minutes / 10 % 10, 0, 0);
  drawChar(minutes % 10, 2, 0);
  drawChar(hours / 10 % 10, 4, 0);
  drawChar(hours % 10, 6, 0);
}