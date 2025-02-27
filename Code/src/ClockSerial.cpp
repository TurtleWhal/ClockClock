#include "ClockSerial.h"
#include "pins.h"

#define INIT_MSG "clock"

ClockSerial::ClockSerial() {}

void ClockSerial::begin()
{
    pinMode(UART_A, INPUT);
    pinMode(UART_B, INPUT);

    Serial1.setPins(UART_A, -1);
    Serial2.setPins(UART_B, -1);

    while (!Serial1.available() && !Serial2.available()) // wait until previous module sends message to determine input and output pins
        delay(100);

    if (Serial1.available())
    {
        if (Serial1.readString() == INIT_MSG)
        {
            _in = UART_A;
            _out = UART_B;
        }
    }
    else if (Serial2.available())
    {
        if (Serial2.readString() == INIT_MSG)
        {
            _in = UART_A;
            _out = UART_B;
        }
    }
    else
        return;

    Serial1.end();
    Serial2.end();

    pinMode(_in, INPUT);
    pinMode(_out, OUTPUT);

    Serial1.setPins(_in, _out);
    Serial1.begin(9600);

    Serial1.write(INIT_MSG);
}

void ClockSerial::handle()
{
    char data[6];
    int index = 0;
    while (Serial1.available() && index < 6)
    {
        data[index++] = Serial1.read();
    }

    if (index == 6)
    {
        _cb(Data((uint64_t)data));
    }
}

void ClockSerial::send(Data data)
{
    for (int i = 0; i < 6; i++)
    {
        Serial1.write((uint8_t)(data.getData() >> (i * 8)));
    }
}

void ClockSerial::onRecieve(void (*cb)(Data)) { _cb = cb; }