#include <Arduino.h>
#include "ClockSerial.h" // includes Data.h
#include "pins.h"

ClockSerial clockSerial;

void handleMessage(Data *data);

uint16_t buffer[8][3][2];

uint8_t map[8][3][2] = {
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
  while (true) {
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
  // clockSerial.send(0b0000000100101101000000000000000000000000000000000000000000000000);
  // delay(1000);
  delay(10000);
  clockSerial.send(new Data());
  delay(10000);
  Data *data = new Data();
  data->setAddress(1);
  clockSerial.send(data);
  delay(25000);

  // put your main code here, to run repeatedly:
  // Serial.println("running");
  // delay(1000);
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

void writeBuffer() {
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 3; j++) {
      clockSerial.send(new Data(map[i][j][0], map[i][j][1], 0, buffer[i][j][0]));
      clockSerial.send(new Data(map[i][j][0], map[i][j][1], 1, buffer[i][j][1]));
    }
  }
}