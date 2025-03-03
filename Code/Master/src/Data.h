class Data {
    private:
        uint64_t _data;

    public:
        Data(uint64_t data = 0b0);
        Data(int address, int face, int hand, int pos);
        uint64_t getData();

        int getAddress();
        void setAddress(int address);

        int getFace();
        void setFace(int face);

        int getHand();
        void setHand(int hand);

        int getPos();
        void setPos(int pos);
};