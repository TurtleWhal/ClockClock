#include <Arduino.h>
#include "ClockModule.h"

#include "SerialTransfer.h"
#include "SPIFFS.h"
#include "Update.h"

// #include "esp_heap_caps.h"

#define BAUDRATE 1000000

void firmwareUpdateTask(void *pvParameters);

ClockModule *m1;
ClockModule *m2;
ClockModule *m3;
ClockModule *m4;

SerialTransfer serialTransfer;

// #define BUFFER_SIZE (1024 * 1024) // 1MB buffer

// uint8_t *largeBuffer = (uint8_t *)heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_SPIRAM);
// uint8_t *largeBuffer;

QueueHandle_t packetQueue;

void setup()
{
  // Serial
  Serial.begin(1000000);
  Serial.println("Helooo");

  packetQueue = xQueueCreate(256 * 10, sizeof(uint8_t));

  // Serial.println("PSRAM before: " + String(ESP.getFreePsram()));
  // largeBuffer = (uint8_t *)ps_malloc(BUFFER_SIZE * sizeof(uint8_t));
  // Serial.println("PSRAM after: " + String(ESP.getFreePsram()));

  // if (!psramInit())
  // {
  //   Serial.println("PSRAM initialization failed!");
  //   return;
  // }

  // delay(1000);

  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // largeBuffer[0] = 12;

  // if (largeBuffer == NULL)
  // {
  //   Serial.println("Failed to allocate buffer in PSRAM!");
  // }
  // else
  // {
  //   Serial.println("Successfully allocated buffer in PSRAM.");
  // }

  int in = 0, out = 0;

  pinMode(UART_A, INPUT_PULLDOWN);
  pinMode(UART_B, INPUT_PULLDOWN);

  Serial1.setPins(UART_A, 26); // pin 26 is one of the headers because it is the only unused pin
  Serial2.setPins(UART_B, 26);

  Serial1.begin(BAUDRATE);
  Serial2.begin(BAUDRATE);

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
  Serial1.setRxBufferSize(1024);
  Serial1.begin(BAUDRATE);

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
    uint16_t recSize = 0;
    uint32_t byte;
    recSize = serialTransfer.rxObj(byte, recSize);

    // Serial.println("Recieved Byte " + String(byte));
    // Serial.println(byte);
    // Firmware Update
    if (serialTransfer.currentPacketID() == 1)
    {
      for (uint8_t i = 4; i < serialTransfer.bytesRead; i++)
      {
        // handle data
        // Serial.print((char)serialTransfer.packet.rxBuff[i]);

        char data = serialTransfer.packet.rxBuff[i];
        // packetQueue.send(&data, sizeof(data));
        xQueueGenericSend(packetQueue, &data, 0, queueSEND_TO_BACK);
        // largeBuffer[byte + (i - 4)] = data;
        // largeBuffer[recievedBytes++] = data;

        // firmware.write(data);
        // Serial.println("Writing: " + String(data));
      }
    }
    else if (serialTransfer.currentPacketID() == 0)
    {
      Serial.println("Recieved firmware size: " + String(firmwareSize) + " bytes");
      // recSize = serialTransfer.rxObj(firmwareSize, recSize);
      // firmware = SPIFFS.open("/firmware.bin", "w");
    }
    else if (serialTransfer.currentPacketID() == 2)
    {
      firmwareUpdate = false;
      Serial.println("Firmware update complete, recieved " + String(firmwareSize) + " bytes");

      // Update.begin(firmwareSize);
      // Update.writeStream(firmware);

      // if (Update.end())
      // {
      //   Serial.println("Successful update");
      // }
      // else
      // {
      //   Serial.println("Error Occurred: " + String(Update.getError()));
      //   return;
      // }

      // firmware.close();

      // Serial.println("Reset in 4 seconds...");
      // delay(4000);

      // ESP.restart();
    }
    // Serial.println();
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

  // Serial.println("LETS GO IT UPDATED OVER SERIAL YEEEEEY (V1.0)");

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
        serialTransfer.rxObj(firmwareSize, recSize);
        firmware = SPIFFS.open("/firmware.bin", "w");
        Serial.println("Recieved firmware size: " + String(firmwareSize) + " bytes");

        xTaskCreatePinnedToCore(
            firmwareUpdateTask,
            "FirmwareUpdateTask",
            8192,
            NULL,
            1,
            NULL,
            0);

        Serial1.onReceive([]()
                          { Serial1.write(Serial1.read()); });
        break;
      }
    }
  }
}

void firmwareUpdateTask(void *pvParameters)
{
  // Serial.println("Firmware update task started");
  // Serial.println("Recieved firmware size: " + String(firmwareSize) + " bytes");

  // uint8_t *buffer = (uint8_t *)malloc(firmwareSize);
  // if (buffer == NULL)
  // {
  //   Serial.println("Failed to allocate buffer for firmware update");
  //   vTaskDelete(NULL);
  // }

  while (recievedBytes < firmwareSize)
  {
    char data;
    xQueueReceive(packetQueue, &data, portMAX_DELAY);
    firmware.write(data);
    recievedBytes++;
    Serial.print((char)data);
  }

  firmware.close();

  Serial1.onReceive(nullptr);

  vTaskDelete(NULL);
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