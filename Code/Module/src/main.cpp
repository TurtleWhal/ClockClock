#include <Arduino.h>
#include "ClockModule.h"
#include "SerialTransfer.h"

ClockModule *m1;
ClockModule *m2;
ClockModule *m3;
ClockModule *m4;

SerialTransfer serialTransfer;

void setup()
{
  // Serial
  Serial.begin(115200);
  Serial.println("Helooo");

  delay(1000);

  int in = 0, out = 0;

  pinMode(UART_A, INPUT_PULLDOWN);
  pinMode(UART_B, INPUT_PULLDOWN);

  Serial1.setPins(UART_A, 26); // pin 26 is one of the headers because it is the only unused pin
  Serial2.setPins(UART_B, 26);

  Serial1.begin(2000000);
  Serial2.begin(2000000);

  Serial1.flush();
  Serial2.flush();

  Serial.println("Listening for input pins");

  bool listening = true;

  while (listening) // wait until previous module sends message to determine input and output pins
  {
    if (Serial1.available() > 10)
    {
      in = UART_A;
      out = UART_B;
      listening = false;
      Serial.println("Using UART_A as input");
    }
    else if (Serial2.available() > 10)
    {
      in = UART_B;
      out = UART_A;
      listening = false;
      Serial.println("Using UART_B as input");
    }
    else
    {
      Serial.print(".");
      delay(100);
    }
  }

  Serial1.end();
  Serial2.end();

  Serial1.flush();

  pinMode(in, INPUT);
  pinMode(out, OUTPUT);

  Serial1.setPins(in, out);
  Serial1.begin(2000000);

  serialTransfer.begin(Serial1);

  uint16_t sendSize = 0;
  sendSize = serialTransfer.txObj((uint8_t)200, sendSize);
  sendSize = serialTransfer.txObj("Hello10Bytes", sendSize);

  serialTransfer.sendData(sendSize);

  m1 = new ClockModule(0);
  m2 = new ClockModule(1);
  m3 = new ClockModule(2);
  m4 = new ClockModule(3);
}

void loop()
{
  m1->minuteStepper->handle();
  m1->hourStepper->handle();

  m2->minuteStepper->handle();
  m2->hourStepper->handle();

  m3->minuteStepper->handle();
  m3->hourStepper->handle();

  m4->minuteStepper->handle();
  m4->hourStepper->handle();

  if (serialTransfer.available())
  {
    uint16_t buffer[24][2];
    uint16_t recSize = 0;
    uint8_t address;
    recSize = serialTransfer.rxObj(address, recSize);

    if (address < 200)
    {
      recSize = serialTransfer.rxObj(buffer, recSize);

      uint16_t sendSize = 0;
      uint8_t sendAddress = address + 1;
      sendSize = serialTransfer.txObj(sendAddress, sendSize);
      sendSize = serialTransfer.txObj(buffer, sendSize);

      serialTransfer.sendData(sendSize);

      Serial.println("Address: " + String(address));

      Serial.print("Buffer: [");
      for (int i = 0; i < 24; i++)
      {
        Serial.print("[" + String(buffer[i][0] / 2) + ", " + String(buffer[i][1] / 2) + "], ");
      }
      Serial.println("]");

      uint8_t ofs = address * 4;

      m1->hourStepper->setTargetPosition(buffer[ofs][0]);
      m1->minuteStepper->setTargetPosition(buffer[ofs][1]);

      m2->hourStepper->setTargetPosition(buffer[ofs + 1][0]);
      m2->minuteStepper->setTargetPosition(buffer[ofs + 1][1]);

      m3->hourStepper->setTargetPosition(buffer[ofs + 2][0]);
      m3->minuteStepper->setTargetPosition(buffer[ofs + 2][1]);

      m4->hourStepper->setTargetPosition(buffer[ofs + 3][0]);
      m4->minuteStepper->setTargetPosition(buffer[ofs + 3][1]);
    }
  }
}

// void handleMessage(Data *data)
// {
//   int address = data->getAddress();
//   // Serial.println(data->getData(), 2);

//   if (address > 0)
//   {
//     data->setAddress(address - 1);
//     clockSerial.send(data);
//   }
//   else
//   {
//     // OMG A MESSAGE JUST FOR ME!!!
//     int face = data->getFace();
//     int hand = data->getHand();
//     int pos = data->getPos();

//     switch (face)
//     {
//     case 0:
//       if (hand == 0)
//         m1->minuteStepper->setTargetPosition(pos);
//       else
//         m1->hourStepper->setTargetPosition(pos);
//       break;
//     case 1:
//       if (hand == 0)
//         m2->minuteStepper->setTargetPosition(pos);
//       else
//         m2->hourStepper->setTargetPosition(pos);
//       break;
//     case 2:
//       if (hand == 0)
//         m3->minuteStepper->setTargetPosition(pos);
//       else
//         m3->hourStepper->setTargetPosition(pos);
//       break;
//     case 3:
//       if (hand == 0)
//         m4->minuteStepper->setTargetPosition(pos);
//       else
//         m4->hourStepper->setTargetPosition(pos);
//       break;
//     }
//   }
// }