#include <Arduino.h>
#include "Data.h"

#define INIT_MSG 0xFE

class ClockSerial
{
private:
    int _in;
    int _out;
    bool _log;
    void (*_cb)(Data *);

    char _data[8]; // Allocate space for 8 bytes
    int _index = 0;

    void handle();
    void printData(uint64_t value, String prefix = "Data");

public:
    ClockSerial();
    void begin(int in, int out, bool log = false);
    void send(Data *data);
    void send(uint64_t data);
    void onRecieve(void (*cb)(Data *));
};