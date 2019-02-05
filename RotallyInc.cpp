#include "RotallyInc.h"

RotaryInc::RotaryInc(PinName pinA,PinName pinB,int md){
    
    if(md%2 == 1){
        measur = true;
        mode = md - 1;
        time = new Timer;
        time->start();
        call = new Timeout;
        call->attach(callback(this,&RotaryInc::zero),1);
        startflag = false;
    }else{
        measur = false;
        mode = md;
    }
    
    A = new InterruptIn(pinA,PullUp);
    B = new InterruptIn(pinB,PullUp);
    A->rise(callback(this,&RotaryInc::riseA));    
        
    if(mode == 2){
        A->fall(callback(this,&RotaryInc::fallA));
    }else if(mode == 4){
        A->fall(callback(this,&RotaryInc::fallA));
        B->rise(callback(this,&RotaryInc::riseB));
        B->fall(callback(this,&RotaryInc::fallB));
    }else{
        mode = 1;
    }
    //time->start();
}

void RotaryInc::zero(){
    speed = 0;
    time->stop();
    time->reset();
    startflag = false;
    count = 0;
}

void RotaryInc::calcu(){
    if(!startflag){
        time->start();
        startflag = true;
        last = pulse;
        count = 1;
        call->attach(callback(this,&RotaryInc::zero),1);
    }else if(count >= 20){
        speed = (double)(pulse - last) / (time->read());
        last = pulse;
        count = 1;
        time->reset();
    }else{
        call->detach();
        count++;
        call->attach(callback(this,&RotaryInc::zero),1);
    }
}

void RotaryInc::riseA(){
    B->read() ? pulse-- : pulse++;
    if(measur)calcu();    
}

void RotaryInc::fallA(){
    B->read() ? pulse++ : pulse--;
    if(measur)calcu();
}

void RotaryInc::riseB(){
    A->read() ? pulse++ : pulse--;
    if(measur)calcu();
}

void RotaryInc::fallB(){
    A->read() ? pulse-- : pulse++;
    if(measur)calcu();
}

long long RotaryInc::get(){
    return pulse;
}

double RotaryInc::getSpeed(){
    return speed / 256 / mode * 314.159265359;//2piR
}

int RotaryInc::diff(){
    int diff = pulse - prev;
    prev = pulse;
    return diff;
}
    
void RotaryInc::reset(){
    pulse = 0;
    prev = 0;
    last = 0;
    if(measur)time->reset();
}

RotaryInc::~RotaryInc(){
    A->disable_irq();
    B->disable_irq();
    delete A;
    delete B;
    if(speed){
        delete time;
        delete call;
    }
}    