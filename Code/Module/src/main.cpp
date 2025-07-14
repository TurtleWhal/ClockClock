#include "Arduino.h"
#include "ClockModule.h"
#include "SerialTransfer.h"
#include "Update.h"
#include "version.h"

#include <rom/gpio.h>
#include "../../Master/src/motorcontrol.h"

#include "pwm.h"

#define BAUDRATE 2000000
#define BUFFER_SIZE (1024 * 1024) // 1MB buffer for FW updates

uint8_t *largeBuffer; // For Firmware Updates, located in PSRAM

ClockModule *modules[4];

SerialTransfer serialTransfer;

int in = 0, out = 0;

void setup()
{
  setCpuFrequencyMhz(240); // Set CPU frequency to 240 MHz

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

  initPWM();

  modules[0] = new ClockModule(0);
  modules[1] = new ClockModule(1);
  modules[2] = new ClockModule(2);
  modules[3] = new ClockModule(3);

  // // spin forever to test motors
  // while (true)
  // {

  //   for (int i = 0; i <= MICRO_STEPS_PER_REVOLUTION; i++)
  //   {
  //     modules[0]->hourStepper->writeStep(i);
  //     modules[0]->minuteStepper->writeStep(i);

  //     modules[1]->hourStepper->writeStep(i);
  //     modules[1]->minuteStepper->writeStep(i);

  //     modules[2]->hourStepper->writeStep(i);
  //     modules[2]->minuteStepper->writeStep(i);

  //     modules[3]->hourStepper->writeStep(i);
  //     modules[3]->minuteStepper->writeStep(i);

  //     delayMicroseconds(4 * 1000000U / MICRO_STEPS_PER_REVOLUTION);
  //   }

  //   delay(1000);
  // }

  // MotorControl_t control = {
  //     .speed = 20,
  //     .direction = MOTOR_CW};

  // for (uint8_t i = 0; i < 4; i++)
  // {
  //   modules[i]->hourStepper->applyMotorControl(control);
  //   modules[i]->minuteStepper->applyMotorControl(control);
  // }

  // while (true)
  // {
  //   vTaskDelay(1000);
  // }
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

      // Firmware Update packet handler
      if (serialTransfer.currentPacketID() == 1)
      {
        for (uint8_t i = 4; i < serialTransfer.bytesRead; i++)
        {
          largeBuffer[recievedBytes] = serialTransfer.packet.rxBuff[i];
          recievedBytes++;
        }
      }
      else if (serialTransfer.currentPacketID() == 2)
      {
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

  if (serialTransfer.available())
  {
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
        modules[i]->hourStepper->applyMotorControl(buffer[ofs + i][0]);
        modules[i]->minuteStepper->applyMotorControl(buffer[ofs + i][1]);
      }
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

        // mirror the uart input to the output so that all data will be forwarded to the next module to firmware update all of them at the same time
        // me and dad spend a whole weekend figuring out these two lines of code
        gpio_matrix_in(in, SIG_IN_FUNC_212_IDX, false);
        gpio_matrix_out(out, SIG_IN_FUNC_212_IDX, false, false);
        break;
      }
    }
  }
}