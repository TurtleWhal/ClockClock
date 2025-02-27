#include <Arduino.h>
// #include "ClockSerial.h"
#include "ClockModule.h"

// ClockSerial clockSerial = ClockSerial();

ClockModule m1 = ClockModule(0);
// ClockModule m2 = ClockModule(1);
// ClockModule m3 = ClockModule(2);
// ClockModule m4 = ClockModule(3);

// void handleMessage(Data data);

void setup()
{
  // Serial
  Serial.begin(115200);
  Serial.println("Helooo");

  // put your setup code here, to run once:
  // clockSerial.onRecieve(handleMessage);
  // clockSerial.begin();

  // m1.hourStepper->setTargetPosition(720);
  m1.minuteStepper->setTargetPosition(720);
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.println("running");
  delay(1000);
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