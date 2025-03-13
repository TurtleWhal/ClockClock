#include <Arduino.h>
#include "Data.h"

#define INIT_MSG 0xC1 // 0xC1OCK

class ClockSerial
{
private:
    int _in;
    int _out;
    void (*_cb)(Data *);

    void handle();

public:
    ClockSerial();
    void begin(int in, int out);
    void send(Data *data);
    void send(uint64_t data);
    void onRecieve(void (*cb)(Data *));
};