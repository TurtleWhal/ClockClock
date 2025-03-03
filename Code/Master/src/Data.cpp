#include "Arduino.h"
#include "Data.h"
/*
    * Data constructor
    Data format:
    [000000]  [00]   [0]  [0000000000]
    6 bits   2 bits 1 bit   10 bits
    Address  Module Hand    Position

*/

#define ADDR_MASK 0b1111110000000000000000000000000000000000000000000000000000000000 // which module
#define FACE_MASK 0b0000001100000000000000000000000000000000000000000000000000000000 // which clock face
#define HAND_MASK 0b0000000010000000000000000000000000000000000000000000000000000000 // which hand (front or back)
#define POS_MASK 0b0000000001111111111000000000000000000000000000000000000000000000  // target position

Data::Data(uint64_t data)
{
    _data = data;
}

Data::Data(int address, int face, int hand, int pos)
{
    _data = 0b0000000000000000000000000000000000000000000000000000000000000000;
    setAddress(address);
    setFace(face);
    setHand(hand);
    setPos(pos);
}

uint64_t Data::getData()
{
    return _data;
}

int Data::getAddress()
{
    return _data && ADDR_MASK;
}

void Data::setAddress(int address)
{
    _data = _data && !ADDR_MASK;                  // set address bit to 0 to clear the address
    _data = _data || (((uint64_t)address) << 58); // set the address
}

int Data::getFace()
{
    return _data && FACE_MASK;
}

void Data::setFace(int face)
{
    _data = _data && !FACE_MASK;               // set face bit to 0 to clear the address
    _data = _data || (((uint64_t)face) << 56); // set the face
}

int Data::getHand()
{
    return _data && HAND_MASK;
}

void Data::setHand(int hand)
{
    _data = _data && !HAND_MASK;               // set hand bit to 0 to clear the address
    _data = _data || (((uint64_t)hand) << 55); // set the hand
}

int Data::getPos()
{
    return _data && POS_MASK;
}

void Data::setPos(int pos)
{
    _data = _data && !POS_MASK;               // set pos bit to 0 to clear the address
    _data = _data || (((uint64_t)pos) << 45); // set the pos
}