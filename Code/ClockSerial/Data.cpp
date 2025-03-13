#include "Arduino.h"
#include "Data.h"
/*
    * Data constructor
    Data format:
    [000000]  [00]   [0]  [0000000000]
    6 bits   2 bits  1 bit  10 bits
    Address  Face   Hand   Position
*/

#define ADDR_MASK 0b1111110000000000000000000000000000000000000000000000000000000000  // 6 bits for address
#define FACE_MASK 0b0000001100000000000000000000000000000000000000000000000000000000  // 2 bits for face
#define HAND_MASK 0b0000000010000000000000000000000000000000000000000000000000000000  // 1 bit for hand
#define POS_MASK  0b0000000001111111111000000000000000000000000000000000000000000000  // 10 bits for position

#define ADDR_SHIFT 58  // Shift by 58 bits to access 6 bits for address
#define FACE_SHIFT 56  // Shift by 56 bits to access 2 bits for face
#define HAND_SHIFT 55  // Shift by 55 bits to access 1 bit for hand
#define POS_SHIFT  45  // Shift by 45 bits to access 10 bits for position

Data::Data(uint64_t data)
{
    _data = data;
}

// Constructor updated to use the smallest data types for the required sizes
Data::Data(uint8_t address, uint8_t face, bool hand, uint16_t pos)
{
    _data = 0;
    setAddress(address);
    setFace(face);
    setHand(hand);
    setPos(pos);
}

uint64_t Data::getData()
{
    return _data;
}

void Data::setData(Data *data)
{
    _data = data->getData();
}

void Data::setData(uint64_t data)
{
    _data = data;
}

// Updated to use uint8_t for address (6 bits)
uint8_t Data::getAddress()
{
    return (_data & ADDR_MASK) >> ADDR_SHIFT;
}

void Data::setAddress(uint8_t address)
{
    _data &= ~ADDR_MASK;  // Clear address bits
    _data |= ((uint64_t)(address & 0x3F) << ADDR_SHIFT);  // Set 6-bit address (0x3F is 6-bit mask)
}

// Updated to use uint8_t for face (2 bits)
uint8_t Data::getFace()
{
    return (_data & FACE_MASK) >> FACE_SHIFT;
}

void Data::setFace(uint8_t face)
{
    _data &= ~FACE_MASK;  // Clear face bits
    _data |= ((uint64_t)(face & 0x03) << FACE_SHIFT);  // Set 2-bit face (0x03 is 2-bit mask)
}

// Updated to use bool for hand (1 bit)
uint8_t Data::getHand()
{
    return (_data & HAND_MASK) >> HAND_SHIFT;
}

void Data::setHand(uint8_t hand)
{
    _data &= ~HAND_MASK;  // Clear hand bit
    _data |= ((uint64_t)(hand & 0x01) << HAND_SHIFT);  // Set 1-bit hand (0x01 is 1-bit mask)
}

// Updated to use uint16_t for pos (10 bits)
uint16_t Data::getPos()
{
    return (_data & POS_MASK) >> POS_SHIFT;
}

void Data::setPos(uint16_t pos)
{
    _data &= ~POS_MASK;  // Clear position bits
    _data |= ((uint64_t)(pos & 0x3FF) << POS_SHIFT);  // Set 10-bit position (0x3FF is 10-bit mask)
}
