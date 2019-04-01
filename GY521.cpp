#include "GY521.h"

GY521::GY521(int bit,int calibration,double user_reg){
    dev_id = 0x68 << 1;
    i2c = new I2C(PB_3,PB_10);
    char check;
    char data[2] = {WHO_AM_I,0x00};
    i2c->write(dev_id,data,1,true);
    i2c->read(dev_id,&check,1);
    if(check == dev_id >> 1){
        data[0] = PWR_MGMT_1;
        i2c->write(dev_id,data,1,true);
        i2c->read(dev_id,&check,1);
        if(check == 0x40){
            i2c->write(dev_id,data,2);
        }
    }
    data[0] = LPF;
    data[1] = 0x00;
    i2c->write(dev_id,data,2);
    short accel_x_now = 0,accel_y_now = 0,accel_z_now = 0;
    double accel_x_aver = 0, accel_y_aver = 0, accel_z_aver = 0;
    data[0] = AFS_SEL;
    data[1] = 0x00;
    i2c->write(dev_id,data,2);
    for (int i = 0; i < calibration; ++i) {
        accel_x_now = gyroRead2(ACCEL_XOUT_H);
        accel_y_now = gyroRead2(ACCEL_YOUT_H);
        accel_z_now = gyroRead2(ACCEL_ZOUT_H);
        accel_x_aver += accel_x_now;
        accel_y_aver += accel_y_now;
        accel_z_aver += accel_z_now;
    }
    accel_x_aver /= calibration;
    accel_y_aver /= calibration;
    accel_z_aver /= calibration;
    double gyro_reg = hypot(hypot(accel_x_aver, accel_y_aver), accel_z_aver) / accel_z_aver;
    data[0] = FS_SEL;
    data[1] = bit << 3;
    i2c->write(dev_id,data,2);
    bit_ = bit;
    gyro_LSB = GY521_LSB_MAP[bit_] / gyro_reg / user_reg;
    
    short gyro_z_now;
    gyro_z_aver = 0;
    for (int i = 0; i < calibration; ++i) {
        gyro_z_now = gyroRead2(GYRO_ZOUT_H);
        gyro_z_aver += gyro_z_now;
    }
    gyro_z_aver /= calibration;

    yaw = diffyaw = 0;
    time.start();
    flag = false;
}

void GY521::updata(){
    if(flag)return;
    short gyro_z_now_ = gyroRead2(GYRO_ZOUT_H);
    gyro_z_now = ((double)gyro_z_now_ - gyro_z_aver) / gyro_LSB;
    diffyaw = (gyro_z_now + gyro_z_prev) / 2 * ((double)time.read_us()/1000000);
    time.reset();
    yaw += diffyaw;
    gyro_z_prev = gyro_z_now;
    //temp = (double)gyroRead2(TEMPERATURE)/340+36.53;
    if (yaw > 225) {
        yaw -= 360;
    } else if (yaw <= -225) {
        yaw += 360;
    }
}

void GY521::reset(int user){
    yaw = user;
    short gyro_z_now;
    flag = true;
    gyro_z_aver = 0;
    for (int i = 0; i < 1000; i++) {
        gyro_z_now = gyroRead2(GYRO_ZOUT_H);
        gyro_z_aver += gyro_z_now;
    }
    gyro_z_aver /= 1000;
    flag = false;
}
    
double GY521::checkStatus(int mode) {
  if (mode == 0) {
    return gyro_LSB;
  } else if (mode == 1) {
    return gyro_z_aver;
  } else if (mode == 2) {
    return acos(gyro_LSB / GY521_LSB_MAP[bit_]);
  }
  return 0;
}

int16_t GY521::gyroRead2(enum GY521RegisterMap reg) {
    char data[2];
    char addr = reg;
    i2c->write(dev_id,&addr,1,true);
    i2c->read(dev_id,data,2);
    return (data[0] << 8) + data[1];
}
