#include "ClockSerial.h"
#include "pins.h"

#define INIT_MSG 0xC1 // 0xC1OCK

ClockSerial::ClockSerial() {}

void ClockSerial::begin()
{

    pinMode(UART_A, OUTPUT);
    pinMode(IC1, INPUT);

    Serial1.setPins(IC1, UART_A);
    Serial1.begin(9600);

    Serial1.write(INIT_MSG); // clear output because for some reason the first message is garbage
    Serial1.flush();

    Serial.println("Broadcasting init message");

    while (Serial1.read() != INIT_MSG)
    {
        Serial1.write(INIT_MSG);
        Serial1.flush();

        // Serial.println(Serial1.read());
        Serial.print(".");
        delay(100);
    }

    Serial.println();

    Serial.println("Serial connection established");

    Serial1.onReceive([this]()
                      { handle(); });
}

void ClockSerial::handle()
{
    char data[8]; // Allocate space for 8 bytes
    int index = 0;

    // Read up to 8 bytes from Serial1
    while (Serial1.available() && index < 8)
    {
        data[index++] = Serial1.read();
    }

    // If we've received exactly 8 bytes, process the data
    if (index == 8)
    {
        uint64_t receivedData = 0;
        for (int i = 0; i < 8; i++)
        {
            receivedData |= (uint64_t)((uint8_t)data[i]) << (i * 8); // Combine the bytes into uint64_t
        }

        // Pass the combined data to the callback function
        _cb(new Data(receivedData));
    }
}

void ClockSerial::send(Data *data)
{
    // Serial.println(data->getData(), 2);
    for (int i = 0; i < 8; i++) // Send 8 bytes for uint64_t
    {
        Serial1.write((uint8_t)(data->getData() >> (i * 8)));
    }
}

void ClockSerial::onRecieve(void (*cb)(Data *)) { _cb = cb; }