#include <Arduino.h>
#include "ClockSerial.h"
#include "ClockModule.h"

ClockSerial clockSerial = ClockSerial();

ClockModule *m1;
ClockModule *m2;
ClockModule *m3;
ClockModule *m4;

void handleMessage(Data *data);

void setup()
{
  // Serial
  Serial.begin(115200);
  Serial.println("Helooo");

  delay(1000);

  int in, out;

  pinMode(UART_A, INPUT);
  pinMode(UART_B, INPUT);

  Serial1.setPins(UART_A, 26); // pin 26 is one of the headers because it is the only unused pin
  Serial2.setPins(UART_B, 26);

  Serial1.begin(9600);
  Serial2.begin(9600);

  Serial.println("Listening for input pins");

  while (!Serial1.available() && !Serial2.available()) // wait until previous module sends message to determine input and output pins
  {
    // Serial.println(Serial1.read());
    Serial.print(".");
    delay(100);
  }

  Serial.println();

  if (Serial1.available())
  {
    Serial1.read(); // clear input because for some reason the first message is garbage
    int msg = Serial1.read();
    Serial.println("Serial1 available: " + String(msg, 16));
    if (msg == INIT_MSG)
    {
      in = UART_A;
      out = UART_B;
      Serial.println("Using UART_A as input");
    }
  }
  else if (Serial2.available())
  {
    Serial2.read(); // clear input because for some reason the first message is garbage
    int msg = Serial2.read();
    Serial.println("Serial2 available: " + String(msg, 16));
    if (msg == INIT_MSG)
    {
      in = UART_B;
      out = UART_A;
      Serial.println("Using UART_B as input");
    }
  }

  Serial1.end();
  Serial2.end();

  m1 = new ClockModule(0);
  m2 = new ClockModule(1);
  m3 = new ClockModule(2);
  m4 = new ClockModule(3);

  // put your setup code here, to run once:
  clockSerial.onRecieve(handleMessage);
  clockSerial.begin(in, out);

  // m1->hourStepper->setTargetPosition(720);
  // m1->minuteStepper->setTargetPosition(720);

  // m2->hourStepper->setTargetPosition(720);
  // m2->minuteStepper->setTargetPosition(720);

  // m3->hourStepper->setTargetPosition(720);
  // m3->minuteStepper->setTargetPosition(720);

  // m4->hourStepper->setTargetPosition(720);
  // m4->minuteStepper->setTargetPosition(720);
}

void loop()
{
  // put your main code here, to run repeatedly:
  // Serial.println("running");
  // delay(1000);

  // static bool flag = false;
  // if (millis() > 10000 && !flag)
  // {
  //   m1->hourStepper->setTargetPosition(180);
  //   // m1->minuteStepper->setTargetPosition(0);
  //   flag = true;
  // }

  m1->minuteStepper->handle();
  m1->hourStepper->handle();

  m2->minuteStepper->handle();
  m2->hourStepper->handle();

  m3->minuteStepper->handle();
  m3->hourStepper->handle();

  m4->minuteStepper->handle();
  m4->hourStepper->handle();

  // for (double i = 0; i < PI * 2; i += PI / 16)
  // {
  //   double sineA = sin(i);
  //   uint8_t raiseA = sineA <= 0 ? 1 : 0;
  //   analogWrite(M1_B3, (sineA + raiseA) * 255);
  //   digitalWrite(M1_B4, raiseA);
  //   // analogWrite(M1_B1, (sineA + raiseA) * 255);
  //   // digitalWrite(M1_B2, raiseA);

  //   double sineB = sin(i + PI / 2);
  //   uint8_t raiseB = sineB <= 0 ? 1 : 0;
  //   analogWrite(M1_B1, (sineB + raiseB) * 255);
  //   digitalWrite(M1_B2, raiseB);
  //   // analogWrite(M1_B1, (sineB + raiseB) * 255);
  //   // digitalWrite(M1_B2, raiseB);

  //   delayMicroseconds(100);
  // }
}

void handleMessage(Data *data)
{
  int address = data->getAddress();
  // Serial.println(data->getData(), 2);

  if (address > 0)
  {
    Data *d = new Data();
    d->setAddress(address - 1);
    clockSerial.send(d);
  }
  else
  {
    // OMG A MESSAGE JUST FOR ME!!!
    int face = data->getFace();
    int hand = data->getHand();
    int pos = data->getPos();

    switch (face)
    {
    case 0:
      if (hand == 0)
        m1->minuteStepper->setTargetPosition(pos);
      else
        m1->hourStepper->setTargetPosition(pos);
      break;
    case 1:
      if (hand == 0)
        m2->minuteStepper->setTargetPosition(pos);
      else
        m2->hourStepper->setTargetPosition(pos);
      break;
    case 2:
      if (hand == 0)
        m3->minuteStepper->setTargetPosition(pos);
      else
        m3->hourStepper->setTargetPosition(pos);
      break;
    case 3:
      if (hand == 0)
        m4->minuteStepper->setTargetPosition(pos);
      else
        m4->hourStepper->setTargetPosition(pos);
      break;
    }
  }
}