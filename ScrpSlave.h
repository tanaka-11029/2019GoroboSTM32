#pragma once
#include "mbed.h"

#define PC 0
#define S_411 1
#define S_446 2

class ScrpSlave{
private:
    uint8_t my_id;
    uint32_t addr;
    bool (*procs[256])(int rx_data, int& tx_data);
    void changeID(uint8_t*);
    bool slave;
    union PCdata{
        double d;
        uint8_t b[8];
    }tx_bits;
    int timeout;
public:
    ScrpSlave(int ser,uint32_t addr);
    ~ScrpSlave();
    DigitalOut* rede;
    Serial *serial;
    FlashIAP *flash;
    void addCMD(uint8_t cmd, bool (*proc)(int rx_data,int& tx_data));
    bool check(void);
    int send(uint8_t,uint8_t,int16_t);
    bool sendPC(int8_t,double,bool indent = false);
    void setTimeout(int);
};