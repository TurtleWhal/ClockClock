class Data {
    private:
        uint64_t _data;

    public:
        Data(uint64_t data = 0b0);
        uint64_t getData();
        int getAddress();
        void setAddress(int address);
};