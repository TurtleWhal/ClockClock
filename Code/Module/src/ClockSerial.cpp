#include "ClockSerial.h"
#include "pins.h"

#define INIT_MSG 0xC1 // 0xC1OCK

ClockSerial::ClockSerial() {}

void ClockSerial::begin()
{
    pinMode(UART_A, INPUT);
    pinMode(UART_B, INPUT);

    Serial1.setPins(UART_A, 26); // pin 26 is one of the headers because it is the only unused pin
    Serial2.setPins(UART_B, 26);

    Serial1.begin(9600);
    Serial2.begin(9600);

    Serial.println("Listening for input pins");

    while (!Serial1.available() && !Serial2.available()) // wait until previous module sends message to determine input and output pins
    {
        // Serial.println(Serial1.read());
        Serial.print(".");
        delay(100);
    }

    Serial.println();

    if (Serial1.available())
    {
        Serial1.read(); // clear input because for some reason the first message is garbage
        int msg = Serial1.read();
        Serial.println("Serial1 available: " + String(msg, 16));
        if (msg == INIT_MSG)
        {
            _in = UART_A;
            _out = UART_B;
            Serial.println("Using UART_A as input");
        }
    }
    else if (Serial2.available())
    {
        Serial2.read(); // clear input because for some reason the first message is garbage
        int msg = Serial2.read();
        Serial.println("Serial2 available: " + String(msg, 16));
        if (msg == INIT_MSG)
        {
            _in = UART_B;
            _out = UART_A;
            Serial.println("Using UART_B as input");
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
    Serial1.flush();

    for (int i = 0; i < 10; i++)
    {
        Serial1.write(INIT_MSG);
        Serial1.flush();
        delay(100);
    }

    Serial.println("Serial communication initialized on TX and RX pins " + String(_in) + " and " + String(_out));

    Serial1.onReceive([this]()
                      { handle(); });
}

void ClockSerial::handle()
{
    char data[8]; // Allocate space for 8 bytes
    int index = 0;
    unsigned long startTime = millis(); // Get current time to implement a timeout

    // Read up to 8 bytes from Serial1, waiting for each byte
    while (index < 8 && (millis() - startTime) < 1000) // 1 second timeout
    {
        if (Serial1.available()) // If a byte is available
        {
            data[index++] = Serial1.read(); // Read the byte and increment index
        }
    }

    // Check if we've received exactly 8 bytes within the timeout
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
    else
    {
        // Handle case where the full 8 bytes were not received (optional)
        Serial.println("Timeout: Did not receive full 8 bytes");
    }
}

void ClockSerial::send(Data *data)
{
    for (int i = 0; i < 8; i++) // Send 8 bytes for uint64_t
    {
        Serial1.write((uint8_t)(data->getData() >> (i * 8)));
    }
}

void ClockSerial::onRecieve(void (*cb)(Data *)) { _cb = cb; }