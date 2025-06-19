#include "Arduino.h"
#include "ClockModule.h"
#include "SerialTransfer.h"
#include "Update.h"
#include "version.h"

#include <rom/gpio.h>
#include <soc/gpio_sig_map.h>
#include "../../Master/src/motorcontrol.h"

#define BAUDRATE 2000000
#define BUFFER_SIZE (1024 * 1024) // 1MB buffer for FW updates

uint8_t *largeBuffer; // For Firmware Updates, located in PSRAM

// ClockModule *m1;
// ClockModule *m2;
// ClockModule *m3;
// ClockModule *m4;

ClockModule *modules[4];

// Define the task function with the correct signature
void MotorUpdateTask(void *pvParameters)
{
  while (true)
  {
    for (int i = 0; i < 4; i++)
    {
      // modules[i]->minuteStepper->update();
      modules[i]->hourStepper->update();
    }
    
    vTaskDelay(1 / portTICK_PERIOD_MS); // Delay to prevent task starvation
  }
}

SerialTransfer serialTransfer;

int in = 0, out = 0;

void setup()
{
  // Serial
  Serial.begin(1000000);
  Serial.print("Helooo. I am a Module. I am V");
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

  pinMode(UART_A, INPUT_PULLDOWN);
  pinMode(UART_B, INPUT_PULLDOWN);

  Serial1.setPins(UART_A, -1); // No output pin, only RX
  Serial2.setPins(UART_B, -1);
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

  // Send init message to next module downstream
  uint16_t sendSize = 0;
  sendSize = serialTransfer.txObj((uint8_t)200, sendSize);
  sendSize = serialTransfer.txObj("Hello10Bytes", sendSize);
  serialTransfer.sendData(sendSize);

  // m1 = new ClockModule(0);
  // m2 = new ClockModule(1);
  // m3 = new ClockModule(2);
  // m4 = new ClockModule(3);

  modules[0] = new ClockModule(0);
  modules[1] = new ClockModule(1);
  modules[2] = new ClockModule(2);
  modules[3] = new ClockModule(3);

  xTaskCreatePinnedToCore(
      MotorUpdateTask,
      "MotorUpdate",
      4096,
      NULL,
      1,
      NULL,
      0); // Run on core 0
}

bool firmwareUpdate = false;
uint32_t firmwareSize = 0;
uint32_t recievedBytes = 0;

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
      // Never called as the 201 address packet handles this
      //   else if (serialTransfer.currentPacketID() == 0)
      //   {
      //    Serial.println("Recieved firmware size: " + String(firmwareSize) + " bytes");
      //     recievedBytes = 0;
      //   }
      else if (serialTransfer.currentPacketID() == 2)
      {
        // send on firmware
        // Serial.println("Forwarding firmware, size: " + String(recievedBytes) + " bytes");
        // uint32_t fileSize = recievedBytes;

        // uint16_t headerSize = 0;

        // uint8_t address = 201;
        // headerSize = serialTransfer.txObj(address, headerSize);
        // headerSize = serialTransfer.txObj(fileSize, headerSize);

        // serialTransfer.sendData(headerSize);

        // uint8_t dataLen = MAX_PACKET_SIZE - 4;
        // uint16_t numPackets = fileSize / dataLen; // Reserve two bytes for current file index

        // if (fileSize % dataLen) // Add an extra transmission if needed
        //   numPackets++;

        // for (uint16_t i = 0; i < numPackets; i++) // Send all data within the file across multiple packets
        // {
        //   uint32_t fileIndex = i * dataLen; // Determine the current file index

        //   if ((fileIndex + dataLen) > fileSize) // Determine data length for the last packet if file length is not an exact multiple of MAX_PACKET_SIZE-2
        //     dataLen = fileSize - fileIndex;

        //   uint8_t sendSize = serialTransfer.txObj(fileIndex);                         // Stuff the current file index
        //   sendSize = serialTransfer.txObj(largeBuffer[fileIndex], sendSize, dataLen); // Stuff the current file data

        //   serialTransfer.sendData(sendSize, 1); // Send the current file index and data
        //   Serial.println("Sending Packet: " + String(i) + " of " + String(numPackets) + " with size: " + String(dataLen) + " bytes");
        //   delay(5); // Needed to not overrun RX buffer
        // }

        // address = 202;
        // uint8_t sendSize = 0;
        // sendSize = serialTransfer.txObj(address, sendSize); // Stuff the current file index

        // serialTransfer.sendData(sendSize, 2); // Send the current file index and data

        // Serial.println("Forwarding Done");

        firmwareUpdate = false;
        Serial.println("Firmware update file recieved, size: " + String(recievedBytes) + " bytes");

        Serial.println("Executing Update");
        if (!Update.begin(recievedBytes))
        {
          Serial.println("Out of Space!");
          return;
        }
        else
          Serial.println("Plenty of Space!");

        Update.write(largeBuffer, recievedBytes);

        if (Update.end())
        {
          Serial.println("Successful update, rebooting...");
          Serial.flush();
          ESP.restart();
        }
        else
        {
          Serial.println("Error Occurred: " + String(Update.getError()));
          Serial.println("Update aborted!");
          return;
        }
      }
    }
    return;
  }

  // m1->minuteStepper->run();
  // m1->hourStepper->run();

  // m2->minuteStepper->run();
  // m2->hourStepper->run();

  // m3->minuteStepper->run();
  // m3->hourStepper->run();

  // m4->minuteStepper->run();
  // m4->hourStepper->run();

  if (serialTransfer.available())
  {
    // uint16_t buffer[24][2];
    MotorControl_t buffer[24][2];
    uint16_t recSize = 0;
    uint8_t address;
    recSize = serialTransfer.rxObj(address, recSize);

    if (address < 200)
    {
      // bool speed;
      // recSize = serialTransfer.rxObj(speed, recSize);

      // bool optimize;
      // recSize = serialTransfer.rxObj(optimize, recSize);

      recSize = serialTransfer.rxObj(buffer, recSize);

      uint16_t sendSize = 0;
      uint8_t sendAddress = address + 1;
      sendSize = serialTransfer.txObj(sendAddress, sendSize);
      // sendSize = serialTransfer.txObj(speed, sendSize);
      // sendSize = serialTransfer.txObj(optimize, sendSize);
      sendSize = serialTransfer.txObj(buffer, sendSize);

      serialTransfer.sendData(sendSize);

      Serial.println("Address: " + String(address));

      Serial.print("Buffer: [");
      for (int i = 0; i < 24; i++)
      {
        Serial.print("[" + String(buffer[i][0].position / 2) + ", " + String(buffer[i][1].position / 2) + "], ");
      }
      Serial.println("]");

      uint8_t ofs = address * 4;

      for (int i = 0; i < 4; i++)
      {
        // modules[i]->hourStepper->setTargetSpeed(buffer[ofs + i][0] - UINT8_MAX);
        // modules[i]->minuteStepper->setTargetSpeed(buffer[ofs + i][1] - UINT8_MAX);
        modules[i]->hourStepper->applyMotorControl(buffer[ofs + i][0]);
        // modules[i]->minuteStepper->applyMotorControl(buffer[ofs + i][1]);
      }

      // if (speed)
      // {
      //   // m1->hourStepper->setTargetSpeed(buffer[ofs][0] - UINT8_MAX);
      //   // m1->minuteStepper->setTargetSpeed(buffer[ofs][1] - UINT8_MAX);

      //   // m2->hourStepper->setTargetSpeed(buffer[ofs + 1][0] - UINT8_MAX);
      //   // m2->minuteStepper->setTargetSpeed(buffer[ofs + 1][1] - UINT8_MAX);

      //   // m3->hourStepper->setTargetSpeed(buffer[ofs + 2][0] - UINT8_MAX);
      //   // m3->minuteStepper->setTargetSpeed(buffer[ofs + 2][1] - UINT8_MAX);

      //   // m4->hourStepper->setTargetSpeed(buffer[ofs + 3][0] - UINT8_MAX);
      //   // m4->minuteStepper->setTargetSpeed(buffer[ofs + 3][1] - UINT8_MAX);

      //   for (int i = 0; i < 4; i++)
      //   {
      //     modules[i]->hourStepper->setTargetSpeed(buffer[ofs + i][0] - UINT8_MAX);
      //     modules[i]->minuteStepper->setTargetSpeed(buffer[ofs + i][1] - UINT8_MAX);
      //   }
      // }
      // else // position
      // {

      //   // m1->hourStepper->setTargetPosition(buffer[ofs][0]);
      //   // m1->minuteStepper->setTargetPosition(buffer[ofs][1]);

      //   // m2->hourStepper->setTargetPosition(buffer[ofs + 1][0]);
      //   // m2->minuteStepper->setTargetPosition(buffer[ofs + 1][1]);

      //   // m3->hourStepper->setTargetPosition(buffer[ofs + 2][0]);
      //   // m3->minuteStepper->setTargetPosition(buffer[ofs + 2][1]);

      //   // m4->hourStepper->setTargetPosition(buffer[ofs + 3][0]);
      //   // m4->minuteStepper->setTargetPosition(buffer[ofs + 3][1]);

      //   for (int i = 0; i < 4; i++)
      //   {
      //     uint16_t posA = buffer[ofs + i][0];
      //     uint16_t posB = buffer[ofs + i][1];

      //     long curA = modules[i]->hourStepper->getCurrentPosition();
      //     long curB = modules[i]->minuteStepper->getCurrentPosition();

      //     int distA = (abs(posA - curA) + abs(posB - curB));
      //     int distB = (abs(posB - curA) + abs(posA - curB));

      //     if (distA < distB || !optimize)
      //     {
      //       modules[i]->hourStepper->setTargetPosition(posA);
      //       modules[i]->minuteStepper->setTargetPosition(posB);
      //     }
      //     else
      //     {
      //       modules[i]->hourStepper->setTargetPosition(posB);
      //       modules[i]->minuteStepper->setTargetPosition(posA);
      //     }
      //   }
      // }
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
        Serial.println("A FW Update Starting, expected size: " + String(firmwareSize) + " bytes");

        uint16_t headerSize = 0;

        uint8_t address = 201;
        headerSize = serialTransfer.txObj(address, headerSize);
        headerSize = serialTransfer.txObj(firmwareSize, headerSize);

        serialTransfer.sendData(headerSize);

        Serial1.flush();

        Serial1.end();
        Serial1.setPins(in, -1);
        Serial1.begin(BAUDRATE);

        gpio_matrix_in(in, SIG_IN_FUNC_212_IDX, false);
        gpio_matrix_out(out, SIG_IN_FUNC_212_IDX, false, false);
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