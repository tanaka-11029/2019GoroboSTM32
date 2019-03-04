#include <ros.h>
#include <std_msgs/Float32MultiArray.h>//フロート型データの配列メッセージ
#include "mbed.h"
#include "ScrpSlave.h"
#include "RotallyInc.h"
#include "GY521.h"

#define MAXPWM 220
#define M_NUM 4

#define R 319.185813605//タイヤの直径 * PI
#define R3 1110//本体の中心からタイヤへの長さの3倍
#define L 260
#define L3 780//中心から計測輪への半径の3倍
#define PI    M_PI
#define PI_2  M_PI_2
#define PI_3  1.047197551196
#define PI_6  0.523598775598
#define PI2_3 2.09439510239

#define Kp 0.0175
#define Ki 0//0.0000135
#define Kd 0//4

#define AMAX 500.0
#define VMAX 3500.0

const PinName PIN[][3] = {
    {PB_14,PB_13,PB_15},//入れ替えた
    {PA_11,PB_1 ,PB_2 },
    {PB_5 ,PB_4 ,PA_10},
    {PC_9 ,PC_8 ,PC_6 }
#if M_NUM > 4
    ,{PB_9 ,PB_8 ,PB_12}
#if M_NUM > 5
    ,{PB_7 ,PB_6 ,PC_7 }
#if M_NUM > 6
    ,{PA_1 ,PA_0 ,PB_0 }
#endif
#endif
#endif
};

const PinName RotaryPin[6][2] = {
    {PA_6 ,PA_7 },
    {PA_14,PA_15},
    {PA_8 ,PA_9 },
    {PC_11,PC_10},//入れ替えた
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
Timer autotimer,motertimer;

ros::NodeHandle nh;

int j,dummy;
bool OK = false;//いっぱい宣言したけどイランやつもあると思う
bool automove = false;
bool PID = false;
bool drivebyms = false;
bool movePID = true;
double prev_t,diff_t,now_t;
double driveMS[3] = {0,0,0};
double t[3],x[3];
double Xmax,Ymax;
double Vmax[3],v[3];
double delta[3];
double X=0,Y=0,T=0;
double Theta,Yaw,Xe,Ye;
double Vx,Vy,Omega;
double Vd[3],nowVx,nowVy,nowVt;
double x_error,y_error,t_error;
double x_diff,y_diff,t_diff;
double driveV[3],nowV[3];

bool safe(int rx_data,int &tx_data){//言わずもがな止める
	if(drivebyms)drivebyms = false;
    Vx = 0;
    Vy = 0;
    Omega = 0;
    for(int i = 0;i<3;i++){
    	driveMS[i] = 0;
    	driveV[i] = 0;
    }
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
        safe(0,dummy);
        Vx = 0;
        Vy = 0;
        Omega = 0;
        gy->reset(0);
        X = 0;
        Y = 0;
        T = 0;
        led.write(1);
    }
}

bool Drive(int id,float pwm){//モーターを回す
    pwm = fmin(fmax(-MAXPWM,pwm),MAXPWM);
    if(!pwm){
        Moter[id][0]->write(0);
        Moter[id][1]->write(0);
        Led[id]->write(0);
    }else if(0 < pwm){
        Moter[id][0]->write(pwm/255);
        Moter[id][1]->write(0);
        Led[id]->write(1);
    }else{
        Moter[id][0]->write(0);
        Moter[id][1]->write(-pwm/255);
        Led[id]->write(1);
    }
    return true;
}
//int Period;
void move(){//X,Y,Omegaから３つのモーターのPWMに変換する
	static const double kp = 0.6,ki = 20,kd = 0.001;//要修正
	static double diff[3],errer[3] = {0,0,0},diffV[3],lastV[3],now_t;
	//static bool flag;
	if(!drivebyms){
		driveMS[0] = Vx*cos(Yaw)         + Vy*sin(Yaw)         + Omega;
		driveMS[1] = Vx*cos(Yaw + PI2_3) + Vy*sin(Yaw + PI2_3) + Omega;
		driveMS[2] = Vx*cos(Yaw - PI2_3) + Vy*sin(Yaw - PI2_3) + Omega;
	}
	if(movePID){
		now_t = motertimer.read();
		motertimer.reset();
	}
	for(j = 0;j < 3;j++){
		nowV[j] = Speed[j]->getSpeed();
		if(movePID){
			diff[j] = driveMS[j] - nowV[j];
			if(nowV[j] == 0 && driveMS[j] == 0){
				errer[j] = 0;
			}
			errer[j] += diff[j] * now_t;
			diffV[j] = (nowV[j] - lastV[j]) / now_t;
			lastV[j] = nowV[j];
			driveV[j] = 0.08 * (driveMS[j] + (diff[j] * kp + errer[j] * ki - diffV[j] * kd));
		}else{
			driveV[j] = driveMS[j];
		}
		Drive(j,driveV[j]);
	}
}

void getData(const std_msgs::Float32MultiArray &msgs){//メッセージ受信時に呼び出される
    switch((int)msgs.data[0]){//メッセージの最初のデータをヘッダーとして使う
    	case -30:
    		safe(0,dummy);
    		break;
    	case -1:
            trigger();
            break;
        case 0://手動走行
            autotimer.stop();
            movePID = true;
        	if(drivebyms)drivebyms = false;
            if(automove)automove = false;
            if(!movePID)movePID = false;
            Vx = msgs.data[1];
            Vy = msgs.data[2];
            Omega = msgs.data[3];
            break;
        case 9:
        	if(!drivebyms)drivebyms = true;
        	if(automove)automove = false;
        	if(movePID)movePID = false;
            driveMS[0] = msgs.data[1];
            driveMS[1] = msgs.data[2];
            driveMS[2] = msgs.data[3];
            break;
        case 10:
        	if(!drivebyms)drivebyms = true;
        	if(automove)automove = false;
        	if(!movePID)movePID = true;
            driveMS[0] = msgs.data[1];
            driveMS[1] = msgs.data[2];
            driveMS[2] = msgs.data[3];
            break;
        case 18://S字加速->PID減速及び微調整
            t[0] = msgs.data[1];//加速時間
            t[1] = msgs.data[2];//加速時間＋並行走行時間
            Xmax = msgs.data[3];//X軸方向の最大速度
            Ymax = msgs.data[4];//Y軸方向の最大速度
            Xe = msgs.data[5];//目標X軸
            Ye = msgs.data[6];//目標Y軸
            Theta = msgs.data[6];//目標向き
            /*Vmax[0] = msgs.data[7];//モーター1の最大速度
            Vmax[1] = msgs.data[8];//モーター2の最大速度
            Vmax[2] = msgs.data[9];//モーター3の最大速度*/
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
            Xe = msgs.data[1];
            Ye = msgs.data[2];
            Theta = msgs.data[3];
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
            safe(0,dummy);
            X = msgs.data[1];
            Y = msgs.data[2];
            gy->reset(msgs.data[3]);
            break;
    }
}

std_msgs::Float32MultiArray now;
ros::Subscriber<std_msgs::Float32MultiArray> sub("moter",&getData);
ros::Publisher place("place", &now);

int main(int argc,char **argv){
    nh.getHardware()->setBaud(115200);//多分最大
    nh.initNode();
    nh.advertise(place);
    nh.subscribe(sub);
    now.data_length = 6;
    now.data = (float *)malloc(sizeof(float)*now.data_length);
    int i;
    double diff[3],Pspeed[3];
    for(i = 0;i<M_NUM;i++){
        Led[i] = new DigitalOut(PIN[i][2],0);
        Moter[i][0] = new PwmOut(PIN[i][0]);
        Moter[i][1] = new PwmOut(PIN[i][1]);
        Moter[i][0]->period_us(2048);
        Moter[i][1]->period_us(2048);
    }
    for(i = 0;i < 3;i++){
        Place[i] = new RotaryInc(RotaryPin[i][0],RotaryPin[i][1],3);
        Speed[i] = new RotaryInc(RotaryPin[i+3][0],RotaryPin[i+3][1],3);
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
    GY521 gyro;
    gy = &gyro;
    loop.reset();
    motertimer.start();
    led.write(1);
    while(1){
        nh.spinOnce();
        gyro.updata();//Yaw軸取得
        Yaw = gyro.yaw;
        if(loop.read_ms() > 20){//10msごとに通信して通信量の調節
            now.data[0] = nowV[0];//X本来はオドメトリを送る
            now.data[1] = nowV[1];//Y
            now.data[2] = nowV[2];//T
            now.data[3] = driveV[0];
            now.data[4] = driveV[1];//mm/s
            now.data[5] = driveV[2];/*
            now.data[6] = T;
            now.data[1] = nowV[1];
            now.data[8] = nowV[1];
            now.data[9] = nowV[2];*/
            //now.data[2] = Period;
            place.publish(&now);
            loop.reset();
        }
        Yaw *= 0.0174532925199432;//pi/180
        move();//モーターの状態を更新
        for(i = 0;i<3;++i){
            diff[i] = Place[i]->diff() / 256.0 * R;
            Pspeed[i] = Place[i]->getSpeed();
        }//オドメトリ計算
        X += -2.0/3.0*diff[0]*cos(Yaw) + 2.0/3.0*diff[1]*cos(Yaw-PI_3) + 2.0/3.0*diff[2]*cos(Yaw+PI_3);
        Y += -2.0/3.0*diff[0]*sin(Yaw) + 2.0/3.0*diff[1]*sin(Yaw-PI_3) + 2.0/3.0*diff[2]*sin(Yaw+PI_3);
        T +=  diff[0]*1/L3 + diff[1]*1/L3 + diff[2]*1/L3;
        nowVx = -2.0/3.0*Pspeed[0]*cos(Yaw) + 2.0/3.0*Pspeed[1]*cos(Yaw-PI_3) + 2.0/3.0*Pspeed[2]*cos(Yaw+PI_3);
        nowVy = -2.0/3.0*Pspeed[0]*sin(Yaw) + 2.0/3.0*Pspeed[1]*sin(Yaw-PI_3) + 2.0/3.0*Pspeed[2]*sin(Yaw+PI_3);
        nowVt =  Pspeed[0]*1/L3 + Pspeed[1]*1/L3 + Pspeed[2]*1/L3;
        if(automove){
        	if(drivebyms)drivebyms = false;
        	if(!movePID)movePID = true;
            now_t = (double)autotimer.read_us() / 1000000;
            if(PID){//なんとなくのPID制御
                diff_t = now_t - prev_t;
                prev_t = now_t;
                x_diff = Xe - X;
                y_diff = Ye - Y;
                t_diff = Theta - Yaw;
                x_error += x_diff * diff_t;
                y_error += y_diff * diff_t;
                t_error += t_diff * diff_t;
                Vx = Kp*x_diff + Ki*x_error - Kd*nowVx;
                Vy = Kp*y_diff + Ki*y_error - Kd*nowVy;
                if(fabs(Vx) > fabs(Xmax)){
                    Vx = Xmax;
                }
                if(fabs(Vy) > fabs(Ymax)){
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
            }else{
                if(now_t < t[0]){//加速だけS字で行う
                    /*for(i = 0;i<3;i++){
                        v[i] = toPWM * (Vmax[i]/2*(1-cos(2*Amax*now_t/Vmax[i])));
                        Drive(i,v[i]);
                    }*/
                    Vx = Xmax/2.0*(1-cos(2.0*AMAX*now_t/Xmax));
                    Vy = Ymax/2.0*(1-cos(2.0*AMAX*now_t/Ymax));
                    Omega = 0;
                }else{
                    PID = true;
                }
            }
        }
    }
}
