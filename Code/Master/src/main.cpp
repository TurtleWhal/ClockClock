#include <Arduino.h>
#include "ClockSerial.h"

ClockSerial clockSerial;

void handleMessage(Data *data);

void setup()
{
  // Serial
  Serial.begin(115200);
  Serial.println("Helooo");

  delay(5000);

  // put your setup code here, to run once:
  clockSerial.onRecieve(handleMessage);
  clockSerial.begin();

  Serial.println((new Data(1, 1, 1, 1))->getData());

  // clockSerial.send(new Data(0b0000000100101101000000000000000000000000000000000000000000000000));
  delay(1000);
  clockSerial.send(new Data(0, 0, 0, 360));
  Serial.println("Send1");
  delay(1000);
  clockSerial.send(new Data(0, 1, 0, 360));
  Serial.println("Send2");
  delay(1000);
  clockSerial.send(new Data(0, 2, 0, 360));
  Serial.println("Send3");
  delay(1000);
  clockSerial.send(new Data(0, 3, 0, 360));
  Serial.println("Send4");
  delay(1000);
}

void loop()
{
  delay(10000);
  // clockSerial.send(new Data());
  // delay(10000);
  // Data *data = new Data();
  // data->setAddress(1);
  // clockSerial.send(data);
  // delay(25000);

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