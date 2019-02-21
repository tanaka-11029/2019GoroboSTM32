#include "ScrpSlave.h"

#define STX 0x41
#define DMY 0xff

ScrpSlave::ScrpSlave(PinName TX1,PinName RX1,uint32_t addr):addr(addr){
    mode = 0;
    init(TX1,RX1);
}

ScrpSlave::ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,uint32_t addr):addr(addr){
    mode = 1;
    rede = new DigitalOut(REDE1);
    init(TX1,RX1);
}

ScrpSlave::ScrpSlave(PinName TX1,PinName RX1,PinName TX2,PinName RX2,uint32_t addr):addr(addr){
    mode = 2;
    serial[1] = new Serial(TX2,RX2,115200);
    serial[1]->attach(callback(this,&ScrpSlave::port2),Serial::RxIrq);
    init(TX1,RX1);
}

ScrpSlave::ScrpSlave(PinName TX1,PinName RX1,PinName REDE1,PinName TX2,PinName RX2,uint32_t addr):addr(addr){
    mode = 3;
    rede = new DigitalOut(REDE1);
    serial[1] = new Serial(TX2,RX2,115200);
    serial[1]->attach(callback(this,&ScrpSlave::port2),Serial::RxIrq);
    init(TX1,RX1);
}

void ScrpSlave::init(PinName TX,PinName RX){
    timeout = 100;
    serial[0] = new Serial(TX,RX,115200);
    serial[0]->attach(callback(this,&ScrpSlave::port1),Serial::RxIrq);
    flash = new FlashIAP;
    if(flash->init()==0){
        if(flash->read(&my_id,addr,1) != 0){
            send(222,222,222);
            my_id = 10;
        }
    }else{
        send(111,111,111);
        my_id = 10;
    }
    for(int i = 1;i<255;++i){
        procs[i] = 0;
    }
}

void ScrpSlave::port1(){
    check(0);
}

void ScrpSlave::port2(){
    check(1);
}

void ScrpSlave::addCMD(uint8_t cmd, bool (*proc)(int rx_data, int& tx_data)){
    if(cmd == 0 || cmd == 254 || cmd == 253)return;
    procs[cmd] = proc;
}

void ScrpSlave::setTimeout(int time){
    timeout = time;
}

void ScrpSlave::changeID(uint8_t id){
    flash->erase(addr,flash->get_sector_size(addr));
    flash->program(&id,addr,1);
}

int ScrpSlave::send(uint8_t id,uint8_t cmd,int16_t tx_data){
    return sending(0,id,cmd,tx_data);
}

int ScrpSlave::send2(uint8_t id,uint8_t cmd,int16_t tx_data){
    if(mode < 2)return -1;
    return sending(1,id,cmd,tx_data);
}

int ScrpSlave::sending(int port,uint8_t id,uint8_t cmd,int16_t tx_data){
    uint8_t tx_dataL = tx_data;
    uint8_t tx_dataH = tx_data >> 8;
    uint8_t tx_sum = id + cmd + tx_dataL + tx_dataH;

    const uint8_t data[] = {DMY, STX, id, cmd, tx_dataL, tx_dataH, tx_sum};
    if(!serial[port]->writeable())return -1;
    if(mode%2 == 1 && id == 0)rede->write(1);
    for(int i = 0;i<7;i++){
        serial[port]->putc(data[i]);
    }
    if(mode%2 == 1 && id == 0)rede->write(0);

    int i = 0;
    bool received = false;
    bool stxflag = false;
    uint8_t rx[5]={},sum = 0;
    Timer out;
    out.start();
    while(out.read_ms() < timeout && !received){
        while(serial[port]->readable()){
            if(serial[port]->getc() == STX && !stxflag){
                stxflag = true;
                continue;
            }
            if(stxflag){
                rx[i] = serial[port]->getc();
                sum += rx[i++];
            }
            if(i > 4){/*
                uint8_t sum = 0;
                for(int j = 0;j<4;j++){
                    sum += rx[j];
                }*/
                if(sum == rx[4]){
                    received = true;
                }
                break;
            }
        }
    }
    out.stop();
    if(!received)return -1;
    return (rx[2] + (rx[3] << 8));
}

void ScrpSlave::check(int id){
    uint8_t rx_cmd;
    int16_t rx_data;
    bool received = false;
    bool broadcast = false;
    while(serial[id]->readable()){
        if(serial[id]->getc() != STX)continue;
        uint8_t rx_id = serial[id]->getc();
        uint8_t tmp_rx_cmd = serial[id]->getc();
        uint8_t tmp_rx_dataL = serial[id]->getc();
        uint8_t tmp_rx_dataH = serial[id]->getc();
        uint8_t rx_sum = serial[id]->getc();
        
        uint8_t sum = rx_id + tmp_rx_cmd + tmp_rx_dataL + tmp_rx_dataH;
        if(sum != rx_sum)continue;
        
        if(rx_id == 255)broadcast = true;
        else if(my_id == rx_id)broadcast = false;
        else break;
        
        rx_cmd = tmp_rx_cmd;
        rx_data = tmp_rx_dataL + ((int16_t)tmp_rx_dataH << 8);
        received = true;
    }
    if(!received) return;
    int tx_data = rx_data;
    if(rx_cmd == 0){
        tx_data = rx_data;
    }else if(rx_cmd == 254){
        uint8_t new_id = rx_data;
        my_id = new_id;
        changeID(new_id);
    }else if(rx_cmd == 253){
        tx_data = my_id;
        rx_cmd = 250;
        broadcast = false;
    }else if(procs[rx_cmd] == 0 || !procs[rx_cmd](rx_data,tx_data))return;
    if(broadcast)return;
    
    uint8_t tx_dataL = tx_data;
    uint8_t tx_dataH = tx_data >> 8;
    uint8_t tx_sum = my_id + rx_cmd + tx_dataL + tx_dataH;
    
    const uint8_t data[] = {DMY, STX, my_id, rx_cmd, tx_dataL, tx_dataH, tx_sum};
    if(!serial[id]->writeable())return;
    if(mode%2 == 1 && id == 0)rede->write(1);
    for(int i = 0;i<7;i++){
        serial[id]->putc(data[i]);
    }
    if(mode%2 == 1 && id == 0)rede->write(0);
    return;
}

ScrpSlave::~ScrpSlave(){
    delete serial[0];
    delete flash;
    if(mode%2 == 1)delete rede;
    if(mode >= 2)delete serial[1];
}
