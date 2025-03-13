class Data
{
private:
    uint64_t _data;

public:
    Data(uint64_t data = 0b0);
    Data(uint8_t address, uint8_t face, bool hand, uint16_t pos);

    uint64_t getData();
    void setData(Data *data);
    void setData(uint64_t data);

    uint8_t getAddress();
    void setAddress(uint8_t address);

    uint8_t getFace();
    void setFace(uint8_t face);

    uint8_t getHand();
    void setHand(uint8_t hand);

    uint16_t getPos();
    void setPos(uint16_t pos);
};