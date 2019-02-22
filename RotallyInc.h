#pragma once
#include "mbed.h"

class RotaryInc{
public:
    RotaryInc(PinName userA, PinName userB,int mode = 0);
    ~RotaryInc();
    long long get();
    double getSpeed();
    void reset(); 
    int diff();
    
private:
    InterruptIn *A,*B;
    Timer *time;
    long long pulse;
    long long last[20];
    long long prev;
    int spend;
    int count;
    int mode;
    double now;
    double sum;
    double pre_t[20];
    double speed;
    bool measur;
    bool startflag;
    bool flag;
    void riseA(void);
    void riseB(void);
    void fallA(void);
    void fallB(void);
    void calcu(void);
    void zero(void);
};
