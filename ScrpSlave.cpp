#include "ScrpSlave.h"

#define STX 0x41
#define DMY 0xff

ScrpSlave::ScrpSlave(int s,uint32_t addr):addr(addr){
    if(s == PC){
        serial = new Serial(SERIAL_TX,SERIAL_RX,115200);
        slave = false;
    }else if(s == S_411){
        serial = new Serial(PA_2,PA_3,115200);
        rede = new DigitalOut(PA_10);
        slave = true;
        timeout = 100;
    }
    flash = new FlashIAP;
    if(flash->init()==0){
        flash->read(&my_id,addr,1);
    }else{
        my_id = 10;
    }
    for(int i = 1;i<255;++i){
        procs[i] = 0;
    }
}

void ScrpSlave::addCMD(uint8_t cmd, bool (*proc)(int rx_data, int& tx_data)){
    if(cmd == 0 || cmd == 254)return;
    procs[cmd] = proc;
}

void ScrpSlave::setTimeout(int time){
    timeout = time;
}

void ScrpSlave::changeID(uint8_t* id){
    flash->erase(addr,1);
    flash->program(id,addr,1);
}

int ScrpSlave::send(uint8_t id,uint8_t cmd,int16_t tx_data){
    uint8_t tx_dataL = tx_data;
    uint8_t tx_dataH = tx_data >> 8;
    uint8_t tx_sum = id + cmd + tx_dataL + tx_dataH;

    const uint8_t data[] = {DMY, STX, id, cmd, tx_dataL, tx_dataH, tx_sum};
    if(!serial->writeable())return -1;
    if(slave)rede->write(1);
    for(int i = 0;i<7;i++){
        serial->putc(data[i]);
    }
    if(slave)rede->write(0);
    
    if(slave){        
        int i;
        bool received = false;
        bool stxflag = false;
        uint8_t rx[5]={};
        Timer out;
        out.start();
        while(out.read_ms() < timeout && !received){
            while(serial->readable()){
                if(serial->getc() == STX && !stxflag){
                    stxflag = true;
                    continue;
                }
                if(stxflag){
                    rx[i++] = serial->getc();
                }
                if(i > 4){
                    uint8_t sum = 0;
                    for(int j = 0;j<4;j++){
                        sum += rx[j];
                    }
                    if(sum == rx[4]){
                        received = true;
                        break;
                    }
                }
            }
        }
        out.stop();
        if(!received)return -1;
        return (rx[2] + (rx[3] << 8));
    }
    return 0;
}

bool ScrpSlave::check(){
    uint8_t rx_cmd;
    int16_t rx_data;
    bool received = false;
    bool broadcast = false;
    while(serial->readable()){
        if(serial->getc() != STX)continue;
        uint8_t rx_id = serial->getc();
        uint8_t tmp_rx_cmd = serial->getc();
        uint8_t tmp_rx_dataL = serial->getc();
        uint8_t tmp_rx_dataH = serial->getc();
        uint8_t rx_sum = serial->getc();
        
        uint8_t sum = rx_id + tmp_rx_cmd + tmp_rx_dataL + tmp_rx_dataH;
        if(sum != rx_sum)continue;
        
        if(rx_id == 255)broadcast = true;
        else if(my_id == rx_id)broadcast = false;
        else continue;
        
        rx_cmd = tmp_rx_cmd;
        rx_data = tmp_rx_dataL + ((int16_t)tmp_rx_dataH << 8);
        received = true;
    }
    if(!received) return false;
    int tx_data = rx_data;
    if(rx_cmd == 0){
        tx_data = rx_data;
    }else if(rx_cmd == 254){
        uint8_t new_id = rx_data;
        my_id = new_id;
        changeID(&new_id);
    }else if(procs[rx_cmd] == 0 || !procs[rx_cmd](rx_data,tx_data))return false;
    if(broadcast)return true;
    
    uint8_t tx_dataL = tx_data;
    uint8_t tx_dataH = tx_data >> 8;
    uint8_t tx_sum = my_id + rx_cmd + tx_dataL + tx_dataH;
    
    const uint8_t data[] = {DMY, STX, my_id, rx_cmd, tx_dataL, tx_dataH, tx_sum};
    if(!serial->writeable())return false;
    if(slave)rede->write(1);
    for(int i = 0;i<7;i++){
        serial->putc(data[i]);
    }
    if(slave)rede->write(0);
    
    return true;
}

ScrpSlave::~ScrpSlave(){
    delete serial;
    delete rede;
    delete flash;
}