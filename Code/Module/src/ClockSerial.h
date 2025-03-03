#include <Arduino.h>
#include "Data.h"

class ClockSerial
{
private:
    int _in;
    int _out;
    void (*_cb)(Data *);

    void handle();

public:
    ClockSerial();
    void begin();
    void send(Data *data);
    void onRecieve(void (*cb)(Data *));
};