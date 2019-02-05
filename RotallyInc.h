#pragma once
#include "mbed.h"

class RotaryInc{
public:
    RotaryInc(PinName userA, PinName userB,int mode);
    ~RotaryInc();
    long long get();
    double getSpeed();
    void reset(); 
    int diff();
    
private:
    InterruptIn *A,*B;
    Timer *time;
    Timeout *call;
    long long pulse;
    long long last;
    long long prev;
    int spend;
    int count;
    int mode;
    double speed;
    bool measur;
    bool startflag;
    void riseA(void);
    void riseB(void);
    void fallA(void);
    void fallB(void);
    void calcu(void);
    void zero(void);
};