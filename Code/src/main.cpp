#include <Arduino.h>
// #include "ClockSerial.h"
#include "ClockModule.h"

// ClockSerial clockSerial = ClockSerial();

ClockModule *m1 = new ClockModule(0);
ClockModule *m2 = new ClockModule(1);
ClockModule *m3 = new ClockModule(2);
ClockModule *m4 = new ClockModule(3);

// void handleMessage(Data data);

void setup()
{
  // Serial
  Serial.begin(115200);
  Serial.println("Helooo");

  // put your setup code here, to run once:
  // clockSerial.onRecieve(handleMessage);
  // clockSerial.begin();

  m1->hourStepper->setTargetPosition(720);
  m1->minuteStepper->setTargetPosition(720);

  m2->hourStepper->setTargetPosition(-720);
  m2->minuteStepper->setTargetPosition(-720);

  m3->hourStepper->setTargetPosition(720);
  m3->minuteStepper->setTargetPosition(720);

  m4->hourStepper->setTargetPosition(-720);
  m4->minuteStepper->setTargetPosition(-720);
}

void loop()
{
  // put your main code here, to run repeatedly:
  // Serial.println("running");
  // delay(1000);

  static bool flag = false;
  if (millis() > 10000 && !flag) {
    m1->hourStepper->setTargetPosition(720*2);
    m1->minuteStepper->setTargetPosition(720*2);
    flag = true;
  }

  m1->minuteStepper->handle();
  // m1->hourStepper->handle();

  m2->minuteStepper->handle();
  // m2->hourStepper->handle();

  m3->minuteStepper->handle();
  // m3->hourStepper->handle();

  m4->minuteStepper->handle();
  // m4->hourStepper->handle();

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

// void handleMessage(Data data)
// {
//   int address = data.getAddress();

//   if (address > 0)
//   {

//     // clockSerial.send(data);
//   }
//   else
//   {
//     // OMG A MESSAGE JUST FOR ME!!!
//   }
// }