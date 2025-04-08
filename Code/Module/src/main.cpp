#include <Arduino.h>
#include "ClockModule.h"
#include "SerialTransfer.h"
#include "SPIFFS.h"
#include "Update.h"
#include "version.h"

#define BAUDRATE 2000000

ClockModule *m1;
ClockModule *m2;
ClockModule *m3;
ClockModule *m4;

SerialTransfer serialTransfer;

#define BUFFER_SIZE (1024 * 1024) // 1MB buffer for FW updates
uint8_t *largeBuffer;

void setup()
{
  // Serial
  Serial.begin(1000000);
  Serial.print("Helooo. I am V");
  Serial.print(VERSION_BUILD);
  Serial.print(". I was built on ");
  Serial.print(VERSION_DATE);
  Serial.print(" at ");
  Serial.println(VERSION_TIME);
  
  log_d("Total heap: %d", ESP.getHeapSize());
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Used PSRAM: %d", ESP.getPsramSize() - ESP.getFreePsram());
  log_d("Free PSRAM: %d", ESP.getFreePsram());
  log_d("Creating Buffer!");

  largeBuffer = (byte *)ps_malloc(BUFFER_SIZE); // For FW updates

  log_d("Used PSRAM: %d", ESP.getPsramSize() - ESP.getFreePsram());
  log_d("Free PSRAM: %d", ESP.getFreePsram());

  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  int in = 0, out = 0;
  pinMode(UART_A, INPUT_PULLDOWN);
  pinMode(UART_B, INPUT_PULLDOWN);

  Serial1.setPins(UART_A, M1_A1); // No output pin, only RX
  Serial2.setPins(UART_B, M2_A1);
  Serial1.begin(BAUDRATE);
  Serial2.begin(BAUDRATE);

  Serial1.flush();
  Serial2.flush();

  Serial.print("Listening for input pins ");

  bool listening = true;

  while (listening) // wait until previous module sends message to determine input and output pins
  {
    if (Serial1.available() > 10)
    {
      in = UART_A;
      out = UART_B;
      listening = false;
      Serial.println("");
      Serial.println("Using UART_A as input");
    }
    else if (Serial2.available() > 10)
    {
      in = UART_B;
      out = UART_A;
      listening = false;
      Serial.println("");
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

  pinMode(in, INPUT);
  pinMode(out, OUTPUT);

  Serial1.setPins(in, out);
  Serial1.setRxBufferSize(1024);
  Serial1.begin(BAUDRATE);
  Serial1.setRxFIFOFull(121u);

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

bool firmwareUpdate = false;
int firmwareSize = 0;
uint32_t recievedBytes = 0;
File firmware;

void loop()
{
  if (firmwareUpdate)
  {
    if (serialTransfer.available())
    {
      uint16_t recSize = 0;
      uint32_t byte;
      recSize = serialTransfer.rxObj(byte, recSize);

      // Firmware Update
      if (serialTransfer.currentPacketID() == 1)
      {
        for (uint8_t i = 4; i < serialTransfer.bytesRead; i++)
        {
          largeBuffer[recievedBytes] = serialTransfer.packet.rxBuff[i];
          recievedBytes++;
        }
      }
      //Never called as the 201 address packet handles this
      // else if (serialTransfer.currentPacketID() == 0)
      // {
      //   Serial.println("Recieved firmware size: " + String(firmwareSize) + " bytes");
      //   recievedBytes = 0;
      // }
      else if (serialTransfer.currentPacketID() == 2)
      {
        firmwareUpdate = false;
        Serial.println("Firmware update complete, recieved " + String(recievedBytes) + " bytes");

        Serial.println("Writing to SPIFFS");
        firmware = SPIFFS.open("/firmware.bin", "wb");
        for (int i = 0; i < recievedBytes; i++)
        {
          firmware.write(largeBuffer[i]);
        }
        firmware.close();
        Serial.println("Done Writing to SPIFFS");

        Serial.println("Start Update");
        firmware = SPIFFS.open("/firmware.bin", "r");
        Serial.println("File Size: " + String(firmware.size()));
        Update.begin(firmware.size());
        Serial.println("Update Size: " + String(Update.writeStream(firmware)));

        if (Update.end())
        {
          Serial.println("Successful update");
          Serial.flush();
          ESP.restart();
        }
        else
        {
          Serial.println("Error Occurred: " + String(Update.getError()));
          return;
        }

      }
    }

    return;
  }

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
    else
    {
      switch (address)
      {
      case 200:
        // sent from previous module to determine input and output pins
        break;
      case 201:
        // start recieving new firmware
        firmwareUpdate = true;
        recievedBytes = 0;
        serialTransfer.rxObj(firmwareSize, recSize);
        Serial.println("Recieved firmware size: " + String(firmwareSize) + " bytes");
        break;
      }
    }
  }
}

// void firmwareUpdateTask(void *pvParameters)
// {
//   // Serial.println("Firmware update task started");
//   // Serial.println("Recieved firmware size: " + String(firmwareSize) + " bytes");

//   // uint8_t *buffer = (uint8_t *)malloc(firmwareSize);
//   // if (buffer == NULL)
//   // {
//   //   Serial.println("Failed to allocate buffer for firmware update");
//   //   vTaskDelete(NULL);
//   // }

//   while (recievedBytes < firmwareSize)
//   {
//     char data;
//     // xQueueReceive(packetQueue, &data, portMAX_DELAY);
//     firmware.write(data);
//     recievedBytes++;
//     // Serial.print((char)data);
//   }

//   firmware.close();

//   Serial1.onReceive(nullptr);

//   vTaskDelete(NULL);
// }

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