#include <ros.h>
#include <sensor_msgs/Joy.h>//ジョイスティックのメッセージを流用
#include <geometry_msgs/Transform.h>//よくわかんないけど使いやすそうなので流用
#include "mbed.h"
#include "ScrpSlave.h"
#include "RotallyInc.h"
#include "GY521.h"

#define MAXPWM 240
#define M_NUM 7

#define R 314.159265359//タイヤの直径 * PI
#define R3 1110//本体の中心からタイヤへの長さの3倍
#define L 260
#define L3 780
#define PI    3.14159265359
#define PI_2  1.57079632679
#define PI_3  1.047197551196
#define PI_6  0.523598775598
#define PI2_3 2.09439510239

#define Kp 0.0175
#define Ki 0.0000135
#define Kd 4

#define AMAX 0.354
#define VMAX 200

#define toPWM 118

const PinName PIN[][3] = {
    {PB_14,PB_13,PB_15},//入れ替えた
    {PA_11 ,PB_1,PB_2 },
    {PA_1 ,PA_0 ,PB_0 },
    {PB_5 ,PB_4 ,PC_4 },
    {PC_9 ,PC_8 ,PC_6 }
#if M_NUM > 5
    ,{PB_7 ,PB_6 ,PC_7 }
#endif
#if M_NUM > 6
    ,{PB_9 ,PB_8 ,PB_12 }
#endif
};

const PinName RotaryPin[6][2] = {
    {PA_7 ,PA_6 },//入れ替えた
    {PA_15,PA_14},
    {PA_9 ,PA_8 },
    {PC_0 ,PC_1 },
    {PC_3 ,PC_2 },
    {PC_5 ,PA_12}
};
/*
const PinName SolenoidPin[] = {
    PC_12,PA_13,PC_14,PC_15,PH_0,PH_1,PC_10,PC_11,PD_2
};
*/
InterruptIn event(PC_13);
DigitalOut led(PA_5);

ScrpSlave arduino(PC_12,PD_2,PA_13,0x0807ffff);

PwmOut* Moter[7][2];
DigitalOut* Led[7];
RotaryInc* Speed[3];
RotaryInc* Place[3];
GY521 *gy;
Timer autotimer;

ros::NodeHandle nh;

int j;
bool OK = false;//いっぱい宣言したけどイランやつもあると思う
bool automove = false;
bool PID = false;
double prev_t,diff_t,now_t;
double t[3],x[3];
double Xmax,Ymax;
double Vmax[3],v[3];
double delta[3];
//double Amax = 0.1;
double X=0,Y=0,T=0;
double Theta,Yaw,Xe,Ye;
double Vx,Vy,Omega;
double Vd[3],nowVx,nowVy,nowVt;
double diff[3];
double x_error,y_error,t_error;
double x_diff,y_diff,t_diff;
double driveV[3];

bool safe(int rx_data,int &tx_data){//言わずもがな止める
    Vx = 0;
    Vy = 0;
    Omega = 0;
    for(int i = 0;i < M_NUM;i++){
        Moter[i][0]->write(0);
        Moter[i][1]->write(0);   
        Led[i]->write(0);
    }
    return true;
}

void trigger(){//ボタンを押されたとき
    if(!OK){
        OK = true;
    }else{
        automove = false;
        led.write(0);
        int dummy;
        safe(0,dummy);
        //gy->reset(0);
        X = 0;
        Y = 0;
        T = 0;
        led.write(1);
    }
}

bool Drive(int id,int pwm){//モーターを回す
    pwm = min(max(-MAXPWM,pwm),MAXPWM);
    if(!pwm){
        Moter[id][0]->write(0);
        Moter[id][1]->write(0);
        Led[id]->write(0);
    }else if(0 < pwm){
        Moter[id][0]->write((float)pwm/255);
        Moter[id][1]->write(0);
        Led[id]->write(1);
    }else{
        Moter[id][0]->write(0);
        Moter[id][1]->write((float)-pwm/255);
        Led[id]->write(1);
    }
    return true;
}

void move(){//X,Y,Omegaから３つのモーターのPWMに変換する
	driveV[0] = Vx*sin(Yaw)         - Vy*cos(Yaw)         + Omega;
	driveV[1] = Vx*sin(Yaw + PI2_3) - Vy*cos(Yaw + PI2_3) + Omega;
	driveV[2] = Vx*sin(Yaw - PI2_3) - Vy*cos(Yaw - PI2_3) + Omega;
	for(j = 0;j < 3;j++){
		Drive(j,driveV[j] + (driveV[j] - toPWM * Speed[j]->getSpeed()));
	}
}

void getData(const sensor_msgs::Joy &msgs){//メッセージ受信時に呼び出される
    switch(msgs.buttons[0]){//Joyメッセージの最初のint方データをヘッダーとして使う
        case -1:
            trigger();
            break;
        case 0://手動走行
            automove = false;
            Vx = (double)msgs.buttons[1];
            Vy = (double)msgs.buttons[2];
            Omega = (double)msgs.buttons[3];
            move();
            break;
        case 10:
            Drive(0,msgs.buttons[1]);
            Drive(1,msgs.buttons[2]);
            Drive(2,msgs.buttons[3]);
            break;
        case 18://S字加速->PID減速及び微調整
            t[0] = msgs.axes[0];//加速時間
            t[1] = msgs.axes[1];//加速時間＋並行走行時間
            Xmax = msgs.axes[2];//X軸方向の最大速度
            Ymax = msgs.axes[3];//Y軸方向の最大速度
            Xe = msgs.axes[4];//目標X軸
            Ye = msgs.axes[5];//目標Y軸
            Theta = msgs.axes[6];//目標向き
            /*Vmax[0] = msgs.axes[7];//モーター1の最大速度
            Vmax[1] = msgs.axes[8];//モーター2の最大速度
            Vmax[2] = msgs.axes[9];//モーター3の最大速度*/
            x_error = 0;
            y_error = 0;
            t_error = 0;
            prev_t = 0;
            diff_t = 0;
            automove = true;
            PID = false;
            autotimer.reset();
            autotimer.start();
            break;
        case 19://PIDだけ
            Xe = msgs.axes[0];
            Ye = msgs.axes[1];
            Theta = msgs.axes[2];
            x_error = 0;
            y_error = 0;
            t_error = 0;
            prev_t = 0;
            diff_t = 0;
            automove = true;
            PID = true;
            autotimer.reset();
            autotimer.start();
            break;
        case 20://停止と補正
            int dummy;
            safe(0,dummy);
            X = msgs.axes[0];
            Y = msgs.axes[1];
            gy->reset(msgs.axes[2]);
            break;
    }
}

geometry_msgs::Transform now;
ros::Subscriber<sensor_msgs::Joy> sub("moter",&getData);
ros::Publisher place("place", &now);

int main(int argc,char **argv){
    nh.getHardware()->setBaud(115200);//多分最大
    nh.initNode();
    nh.advertise(place);
    nh.subscribe(sub);
    int i;
    for(i = 0;i<M_NUM;i++){
        Led[i] = new DigitalOut(PIN[i][2],0);
        Moter[i][0] = new PwmOut(PIN[i][0]);
        Moter[i][1] = new PwmOut(PIN[i][1]);
        Moter[i][0]->period_us(2048);
        Moter[i][1]->period_us(2048);
    }
    for(i = 0;i < 3;i++){
        Place[i] = new RotaryInc(RotaryPin[i][0],RotaryPin[i][1],0);
        Speed[i] = new RotaryInc(RotaryPin[i+3][0],RotaryPin[i+3][1],1);
    }
    event.rise(&trigger);
    Timer loop;
    loop.start();
    while(!OK){//キャリブレーション待機
        nh.spinOnce();
        if(loop.read_ms() > 250){
            led = !led;
            loop.reset();
        }
    }
    led.write(0);
    for(i = 0;i<3;i++){
        Speed[i]->reset();
        Place[i]->reset();
    }
    //GY521 gyro;
    //gy = &gyro;
    loop.reset();
    led.write(1);
    while(1){
        nh.spinOnce();
        //gyro.updata();//Yaw軸取得
        Yaw = 0;//gyro.yaw;
        if(loop.read_ms() > 30){//10msごとに通信して通信量の調節
            now.rotation.x = Vx;//X本来はオドメトリを送る
            now.rotation.y = Vy;//Y
            now.rotation.z = Place[1]->get();//T
            now.rotation.w = Yaw;
            now.translation.x = Speed[0]->getSpeed();
            now.translation.y = Speed[1]->getSpeed();
            now.translation.z = Speed[2]->getSpeed();
            place.publish(&now);
            loop.reset();
        }
        Yaw *= 0.0174532925199432;//pi/180
        
        for(i = 0;i<3;++i){
            diff[i] = Place[i]->diff() / 256 * R;
        }//オドメトリ計算
        X += -2/3*diff[0]*sin(Yaw) + 2/3*diff[1]*sin(Yaw-PI_3) + 2/3*diff[2]*sin(Yaw+PI_3);
        Y +=  2/3*diff[0]*cos(Yaw) - 2/3*diff[1]*cos(Yaw-PI_3) - 2/3*diff[2]*cos(Yaw+PI_3);
        T += diff[0]*1/L3 + diff[1]*1/L3 + diff[2]*1/L3;
        nowVx =  2/3*Speed[0]->getSpeed()*sin(Yaw) - 2/3*Speed[1]->getSpeed()*sin(Yaw-PI_3) - 2/3*Speed[2]->getSpeed()*sin(Yaw+PI_3);
        nowVy = -2/3*Speed[0]->getSpeed()*cos(Yaw) + 2/3*Speed[1]->getSpeed()*cos(Yaw-PI_3) + 2/3*Speed[2]->getSpeed()*cos(Yaw+PI_3);
        if(automove){
            now_t = (double)autotimer.read_us() / 1000;
            if(PID){//なんとなくのPID制御
                diff_t = now_t - prev_t;
                prev_t = now_t;
                x_diff = Xe - X;
                y_diff = Ye - Y;
                t_diff = Theta - Yaw;
                x_error += x_diff * diff_t;
                y_error += y_diff * diff_t;
                t_error += t_diff * diff_t;
                nowVt = Speed[0]->getSpeed()*1/R3 + Speed[1]->getSpeed()*1/R3 + Speed[2]->getSpeed()*1/R3;
                if(now_t >= t[1] || abs(Kp*x_diff + Ki*x_error - Kd*nowVx) <= abs(Xmax) || abs(Kp*y_diff + Ki*y_error - Kd*nowVy) <= abs(Ymax)){
                    Vx = Kp*x_diff + Ki*x_error - Kd*nowVx;
                    Vy = Kp*y_diff + Ki*y_error - Kd*nowVy;
                }else{
                    Vx = Xmax;
                    Vy = Ymax;
                }
                Omega = Kp*t_diff + Ki*t_error - Kd*nowVt;
                if(Omega > 50)Omega = 50;
                else if(Omega < -50)Omega = -50;
                if(x_diff < 30 && y_diff < 30 && t_diff < PI/90){
                    automove = false;
                    Vx = 0;
                    Vy = 0;
                    Omega = 0;
                }
                move();
            }else{
                if(now_t < t[0]){//加速だけS字で行う
                    /*for(i = 0;i<3;i++){
                        v[i] = toPWM * (Vmax[i]/2*(1-cos(2*Amax*now_t/Vmax[i])));
                        Drive(i,v[i]);
                    }*/
                    Vx = Xmax/2*(1-cos(2*AMAX*now_t/Xmax));
                    Vy = Ymax/2*(1-cos(2*AMAX*now_t/Ymax));
                    Omega = 0;
                    move();
                }else{
                    PID = true;
                }
            }
        }
    }
}
