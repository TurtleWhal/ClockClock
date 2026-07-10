#include "Arduino.h"
#include "ClockModule.h"
#include "SerialTransfer.h"
#include "Update.h"
#include "esp_task_wdt.h"
#include "version.h"

#include "../../Master/src/motorcontrol.h"
#include <rom/gpio.h>

#include "mcpwm.h"
#include "pwm.h"

#include "NewStepper.h"

#define BAUDRATE 2000000
#define BUFFER_SIZE (1024 * 1024) // 1MB buffer for FW updates

uint8_t *largeBuffer; // For Firmware Updates, located in PSRAM

ClockModule *modules[4];

SerialTransfer serialTransfer;

int in = 0, out = 0;

void serialTask(void *);

void setup() {
  setCpuFrequencyMhz(240); // Set CPU frequency to 240 MHz

  // Serial
  Serial.begin(1000000);
  Serial.print("Helooo. I am a Module. I am V");
  Serial.print(VERSION_BUILD);
  Serial.print(". I was built on ");
  Serial.print(VERSION_DATE);
  Serial.print(" at ");
  Serial.println(VERSION_TIME);

  // The PWM busy loop (core 1) and the serial loop (core 0) both run without
  // yielding, each starving its core's idle task. The Task WDT monitors core
  // 0's idle by default, which would panic ~5 s after the serial task starts.
  // Stop the WDT from watching any idle task (idle_core_mask = 0). This is
  // consistent with the firmware's design of dedicating cores to non-yielding
  // loops; INT_WDT and esp_timer-based stepping are unaffected.
  esp_task_wdt_config_t wdtConfig = {
      .timeout_ms = 5000,
      .idle_core_mask = 0, // do not monitor either core's idle task
      .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&wdtConfig);

  log_d("Total heap: %d", ESP.getHeapSize());
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Used PSRAM: %d", ESP.getPsramSize() - ESP.getFreePsram());
  log_d("Free PSRAM: %d", ESP.getFreePsram());
  log_d("Creating Buffer!");

  largeBuffer = (byte *)ps_malloc(BUFFER_SIZE); // For FW updates

  log_d("Used PSRAM: %d", ESP.getPsramSize() - ESP.getFreePsram());
  log_d("Free PSRAM: %d", ESP.getFreePsram());

#ifndef MOTOR_TEST

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

  while (listening) // wait until previous module sends message to determine
                    // input and output pins
  {
    if (Serial1.available() > 10) {
      in = UART_A;
      out = UART_B;
      listening = false;
      Serial.println("");
      Serial.println("Using UART_A as input");
    } else if (Serial2.available() > 10) {
      in = UART_B;
      out = UART_A;
      listening = false;
      Serial.println("");
      Serial.println("Using UART_B as input");
    } else {
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

  // Run the serial loop on core 0, leaving core 1 to the PWM busy loop. Stack
  // matches the Arduino loop task (the firmware-update path needs the
  // headroom).
  xTaskCreatePinnedToCore(serialTask, "SerialTask", 8192, NULL, 1, NULL, 0);

#else

  NewStepper *motor1 = new NewStepper(M4_A1, M4_A2, M4_A3, M4_A4, MICRO_STEP_MCPWM);
  NewStepper *motor2 = new NewStepper(M4_B3, M4_B4, M4_B1, M4_B2, MICRO_STEP_LEDC);

  // Only the two magnitude legs per motor (pin1A/pin2A) go to MCPWM. The
  // direction legs (pin1B/pin2B) MUST stay plain GPIO so digitalWrite in
  // writeMicrostep can flip coil polarity; bind them to MCPWM and they're stuck
  // at idle duty, the field never reverses, and the motor only vibrates.
  // Called after the constructors so MCPWM owns the pad last.
  mcpwmInit((uint8_t[]){M4_A1, M4_A3, M4_B3, M4_B1}, 4);

  while (true) {
    motor1->microstep();
    motor2->microstep();
    delayMicroseconds(10 * (1000000U / MICRO_STEPS_PER_REVOLUTION));
  }

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
#endif
}

bool firmwareUpdate = false;
uint32_t firmwareSize = 0;
uint32_t recievedBytes = 0;

// All serial RX/parse/forward and the firmware-update cascade. Runs in its own
// task on core 0 (see setup) so it stays off core 1, which the PWM busy loop
// owns. esp_timer-based stepping also lives on core 0 and just briefly preempts
// this at higher priority — that's fine; serial RX is FIFO/ring-buffered.
void serialLoopBody() {
  if (firmwareUpdate) {
    if (serialTransfer.available()) {
      uint16_t recSize = 0;
      uint32_t byte;
      recSize = serialTransfer.rxObj(byte, recSize);

      // Firmware Update packet handler
      if (serialTransfer.currentPacketID() == 1) {
        for (uint8_t i = 4; i < serialTransfer.bytesRead; i++) {
          largeBuffer[recievedBytes] = serialTransfer.packet.rxBuff[i];
          recievedBytes++;
        }
      } else if (serialTransfer.currentPacketID() == 2) {
        firmwareUpdate = false;
        Serial.println("Firmware update file recieved, size: " +
                       String(recievedBytes) + " bytes");

        Serial.println("Executing Update");
        if (!Update.begin(recievedBytes)) {
          Serial.println("Out of Space!");
          return;
        } else
          Serial.println("Plenty of Space!");

        Update.write(largeBuffer, recievedBytes);

        if (Update.end()) {
          Serial.println("Successful update, rebooting...");
          Serial.flush();
          ESP.restart();
        } else {
          Serial.println("Error Occurred: " + String(Update.getError()));
          Serial.println("Update aborted!");
          return;
        }
      }
    }
    return;
  }

  if (serialTransfer.available()) {
    uint16_t recSize = 0;
    uint8_t address;
    recSize = serialTransfer.rxObj(address, recSize);

    if (address < 200) {
      MotorControl_t buffer[4][2];
      recSize = serialTransfer.rxObj(buffer, recSize);

      if (address > 0) {
        // forward packets to next module

        uint16_t sendSize = 0;
        uint8_t sendAddress = address - 1;
        sendSize = serialTransfer.txObj(sendAddress, sendSize);
        sendSize = serialTransfer.txObj(buffer, sendSize);

        Serial.printf("Recieved for %d, forwarding to %d\n", address,
                      sendAddress);

        serialTransfer.sendData(sendSize);
      } else {
        // address 0 mean it is for me
        Serial.print("Buffer: [");
        for (int i = 0; i < 4; i++) {
          Serial.print("[" + String(buffer[i][0].position) + ", " +
                       String(buffer[i][1].position) + "]" +
                       (i < 3 ? ", " : ""));
        }
        Serial.println("]");

        for (int i = 0; i < 4; i++) {
          if (buffer[i][0].optimize && buffer[i][1].optimize) {
            uint16_t distA =
                abs(modules[i]->hourStepper->getCurrentPosition() -
                    buffer[i][0].position) +
                abs(modules[i]->minuteStepper->getCurrentPosition() -
                    buffer[i][1].position);
            uint16_t distB =
                abs(modules[i]->minuteStepper->getCurrentPosition() -
                    buffer[i][0].position) +
                abs(modules[i]->hourStepper->getCurrentPosition() -
                    buffer[i][1].position);

            if (distB < distA) {
              // swap positions
              MotorControl_t temp = buffer[i][0];
              buffer[i][0] = buffer[i][1];
              buffer[i][1] = temp;
            }
          }

          modules[i]->hourStepper->applyMotorControl(buffer[i][0]);
          modules[i]->minuteStepper->applyMotorControl(buffer[i][1]);
        }
      }
    } else {
      switch (address) {
      case 200:
        // sent from previous module to determine input and output pins
        break;
      case 201: {
        // start recieving new firmware
        firmwareUpdate = true;
        recievedBytes = 0;
        serialTransfer.rxObj(firmwareSize, recSize);
        Serial.println("A FW Update Starting, expected size: " +
                       String(firmwareSize) + " bytes");

        uint16_t headerSize = 0;

        uint8_t address = 201;
        headerSize = serialTransfer.txObj(address, headerSize);
        headerSize = serialTransfer.txObj(firmwareSize, headerSize);

        serialTransfer.sendData(headerSize);

        Serial1.flush();

        Serial1.end();
        Serial1.setPins(in, -1);
        Serial1.begin(BAUDRATE);

        // mirror the uart input to the output so that all data will be
        // forwarded to the next module to firmware update all of them at the
        // same time me and dad spend a whole weekend figuring out these two
        // lines of code
        gpio_matrix_in(in, SIG_IN_FUNC_212_IDX, false);
        gpio_matrix_out(out, SIG_IN_FUNC_212_IDX, false, false);
        break;
      }
      case 220:
        // Calibration command, zeros motors to straight down (90°)
        for (int i = 0; i < 4; i++) {
          modules[i]->hourStepper->setPosition(90);
          modules[i]->minuteStepper->setPosition(90);
        }
        break;
      }
    }
  }
}

void serialTask(void *) {
  for (;;)
    serialLoopBody();
}

// The Arduino loopTask runs on core 1, which the PWM busy loop owns. Keep it
// dormant so it never steals cycles from PWM; all real work runs in serialTask
// (core 0) and the esp_timer stepping callbacks.
void loop() { vTaskDelay(portMAX_DELAY); }