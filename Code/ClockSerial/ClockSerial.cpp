#include "ClockSerial.h"

ClockSerial::ClockSerial() {}

void ClockSerial::begin(int in, int out)
{
    _in = in;
    _out = out;

    pinMode(_in, INPUT);
    pinMode(_out, OUTPUT);

    Serial1.setPins(_in, _out);
    Serial1.begin(9600);

    Serial1.write(INIT_MSG);
    Serial1.flush();

    for (int i = 0; i < 10; i++) // broadcast for 1 second
    {
        Serial1.write(INIT_MSG);
        Serial1.flush();
        delay(100);
    }

    Serial.println("Serial communication initialized on TX and RX pins " + String(_in) + " and " + String(_out));

    Serial1.onReceive([this]()
                      { handle(); });
}

// Function to log the full 64-bit number in binary
void printData(uint64_t value, String prefix = "Data")
{
    Serial.print(prefix + ": ");
    for (int i = 63; i >= 0; i--)
    {
        Serial.print((value >> i) & 0x01); // Extract and print each bit of the 64-bit value
        if (i == 64 - 6 || i == 64 - 8 || i == 64 - 9 || i == 64 - 19)
            Serial.print(' '); // Space between bytes for readability
    }
    Serial.println();
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
            // Reverse the order by shifting the least significant byte first
            receivedData |= (uint64_t)((uint8_t)data[i]) << (i * 8); // Combine the bytes into uint64_t
        }

        printData(receivedData, "Received");

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
    printData(data->getData(), "Sending Data");
    for (int i = 0; i < 8; i++) // Send 8 bytes for uint64_t
    {
        Serial1.write((uint8_t)(data->getData() >> (i * 8)));
    }
}

void ClockSerial::send(uint64_t data)
{
    printData(data, "Sending Bytes");
    for (int i = 0; i < 8; i++)
    {
        uint8_t byteToSend = (data >> (8 * (7 - i))) & 0xFF; // Extract each byte
        Serial1.write(byteToSend);                           // Send the byte
    }
}

void ClockSerial::onRecieve(void (*cb)(Data *)) { _cb = cb; }