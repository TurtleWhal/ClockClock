#include "Arduino.h"
#include "Data.h"
/*
    * Data constructor
    Data format:
    [000000][]
    6 bits
    address
    
*/

#define ADDR_MASK 0b1111110000000000000000000000000000000000000000000000000000000000

Data::Data(uint64_t data)
{
    _data = data;
}

int Data::getAddress()
{
    return _data && ADDR_MASK;
}

void Data::setAddress(int address)
{
    _data = _data && !ADDR_MASK; // set address bit to 0 to clear the address
    _data = _data || (address << 58); // set the address
}

uint64_t Data::getData()
{
    return _data;
}