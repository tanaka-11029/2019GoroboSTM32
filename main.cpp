#include <ros.h>
#include <std_msgs/Float32MultiArray.h>//フロート型データの配列メッセージ
#include "mbed.h"
#include "ScrpSlave.h"
#include "RotallyInc.h"
#include "GY521.h"

#define MAXPWM 220
#define M_NUM 5

#define R 319.185813605//タイヤの直径 * PI
#define R3 1110//本体の中心からタイヤへの長さの3倍
#define L 260
#define L3 780//中心から計測輪への半径の3倍
#define PI    M_PI
#define PI_2  M_PI_2
#define PI_3  1.047197551196
#define PI_6  0.523598775598
#define PI2_3 2.09439510239

#define Kp 1.6//移動
#define Ki 0.2
#define Kd 0.001

#define kp 0.048//モーター
#define ki 1.6
#define kd 0.00008

#define AMAX 400.0
#define VMAX 400.0

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

const PinName OutPin[5] = {
    PA_1,PA_0,PB_0,PB_6,PC_0
};

const PinName InPin[6] = {
	PC_1,PC_4,PA_4,PH_1,PB_7,PC_7
};

InterruptIn event(PC_13);
DigitalOut led(PA_5);

//ScrpSlave arduino(PC_12,PD_2,PA_13,0x0807ffff);

PwmOut* Moter[7][2];
DigitalOut* Led[7];
DigitalOut* Solenoid[5];
DigitalIn* test[4];
DigitalIn* Limit[6];
RotaryInc* Speed[3];
RotaryInc* Place[3];
GY521 *gy;
Timer autotimer,motertimer,kikoutimer;

ros::NodeHandle nh;

uint16_t kikou = 0;
int j;
bool OK = false;//いっぱい宣言したけどイランやつもあると思う
bool automove = false;
bool autoX = false;
bool autoY = false;
bool autoT = false;
bool drivebyms = false;
bool limit = false;
bool movePID = true;
bool flag = false;
bool drivef1,driveb1,drivef2,driveb2;
double stampX,stampY;
double prev_t,diff_t,now_t;
double driveMS[3] = {0,0,0};
double t[3];
double Xmax,Ymax;
double X=0,Y=0,T=0;
double Theta,Yaw,Xe,Ye;
double Vx,Vy,Omega;
double nowVx,nowVy,nowVt;
double x_error,y_error,t_error;
double x_diff,y_diff,t_diff;
double driveV[3],nowV[3];

void safe(){//言わずもがな止める
    Vx = 0;
    Vy = 0;
    Omega = 0;
    automove = false;
    autoX = false;
    autoY = false;
    autoT = false;
    kikou = 0;
    limit = false;
	flag = false;
	kikoutimer.stop();
	kikoutimer.reset();
    if(drivebyms)drivebyms = false;
    for(int i = 0;i<3;i++){
    	driveMS[i] = 0;
    	driveV[i] = 0;
    }
    for(int i = 0;i < M_NUM;i++){
        Moter[i][0]->write(0);
        Moter[i][1]->write(0);   
        Led[i]->write(0);
    }
	for(int i = 0;i < 4;i++){
		Solenoid[i]->write(0);
	}
}

void trigger(){//ボタンを押されたとき
    if(!OK){
        OK = true;
    }else{
        led.write(0);
        safe();
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

void move(){//X,Y,Omegaから３つのモーターのPWMに変換する
	static double diff[3],errer[3] = {0,0,0},diffV[3],lastV[3],now_t;
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
			if(nowV[j] == 0 && driveMS[j] == 0 && errer[j] != 0){
				errer[j] = 0;
			}
			errer[j] += diff[j] * now_t;
			diffV[j] = (nowV[j] - lastV[j]) / now_t;
			lastV[j] = nowV[j];
			driveV[j] = 0.08 * driveMS[j] + diff[j] * kp + errer[j] * ki - diffV[j] * kd;
		}else{
			driveV[j] = driveMS[j];
		}
		Drive(j,driveV[j]);
	}
}

void getData(const std_msgs::Float32MultiArray &msgs){//メッセージ受信時に呼び出される
    switch((int)msgs.data[0]){//メッセージの最初のデータをヘッダーとして使う
    	case -40:
    		kikou = 0;
    		flag = false;
    		kikoutimer.stop();
    		kikoutimer.reset();
    		Drive(3,0);
    		Drive(4,0);
    		for(int i = 0;i < 4;i++){
    			Solenoid[i]->write(0);
    		}
    		break;
    	case -30:
    		safe();
    		break;
    	case -31:/*
    		if(Limit[1]->read() && Limit[2]->read()){
    			Drive(3,0);
    			drivef1 = false;
    		}else{
    			drivef1 = true;*/
    			Drive(3,-90);//ただただペットボトルを前に出す
    		//}
    		break;
    	case -32:
    		if(Limit[3]->read()){
    			Drive(3,0);
    			driveb1 = false;
    		}else{
    			driveb1 = true;
    			Drive(3,90);//ただただペットボトルを戻す
    		}
    		break;
    	case -33:
    		Drive(3,0);//止める
    		drivef1 = false;
    		driveb1 = false;
    		break;
    	case -34:
    		if(!Limit[4]->read()){
    			Drive(4,0);
    			drivef2 = false;
    		}else{
    			Drive(4,30);
    			drivef2  = true;
    			driveb2 = false;
    		}
    		break;
    	case -35:
    		if(!Limit[5]->read()){
    			Drive(4,0);
    			driveb2 = false;
    		}else{
    			Drive(4,-30);
    			driveb2 = true;
    			drivef2 = false;
    		}
    		break;
    	case -36:
    		Drive(4,0);
    		drivef2 = false;
    		driveb2 = false;
    		break;
    	case -37:
    		Solenoid[2]->write(1);
    		break;
    	case -38:
    		Solenoid[2]->write(0);
    		break;
    	case -10://ペットボトルを取る
    		kikou |= 1;
    		break;
    	case -11://ペットボトルを置く
    		kikou |= 2;
    		break;
    	case -12://食事を取る、下げる
    		kikou |= 4;
    		break;
    	case -13://食事を取る、上げる
    		kikou |= 8;
    		break;
    	case -14://食事を投げる
    		kikou |= 16;
    		break;
    	case -15://ペットボトル機構を前に出す
    		kikou |= 32;
    		break;
    	case -16://ペットボトル機構を戻す
    		kikou |= 64;
    		break;
    	case -17://ペットボトルをつかむ
    		kikou |= 128;
    		break;
    	case -18://ペットボトルをはなす
    		kikou |= 256;
    		break;
    	case -1:
            trigger();
            break;
        case 0://手動走行
            autotimer.stop();
            movePID = true;
            if(limit)limit = false;
            if(autoX)autoX = false;
            if(autoY)autoY = false;
            if(autoT)autoT = false;
        	if(drivebyms)drivebyms = false;
            if(automove)automove = false;
            if(!movePID)movePID = true;
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
        case 15:
        	t[0] = msgs.data[1];
        	Xmax = msgs.data[2];
        	Xe = msgs.data[3];
            x_error = 0;
        	autoX = true;
        	limit = false;
        	if(automove){
        		stampX = now_t;
        	}else{
        		stampX = 0;
                prev_t = 0;
                diff_t = 0;
        		automove = true;
                autotimer.reset();
                autotimer.start();
        	}
        	break;
        case 16:
        	t[1] = msgs.data[1];
        	Ymax = msgs.data[2];
        	Ye = msgs.data[3];
        	limit = msgs.data[4];
            y_error = 0;
        	autoY = true;
        	if(automove){
        		stampY = now_t;
        	}else{
        		stampY = 0;
                prev_t = 0;
                diff_t = 0;
        		automove = true;
                autotimer.reset();
                autotimer.start();
        	}
        	break;
        case 17:
        	Theta = msgs.data[1];
            t_error = 0;
            autoT = true;
            limit = false;
        	if(!automove){
        		automove = true;
                prev_t = 0;
                diff_t = 0;
                autotimer.reset();
                autotimer.start();
        	}
        	break;
        case 18://S字加速->PID減速及び微調整
            t[0] = msgs.data[1];//加速時間
            t[1] = msgs.data[2];//加速時間＋並行走行時間
            Xmax = msgs.data[3];//X軸方向の最大速度
            Ymax = msgs.data[4];//Y軸方向の最大速度
            Xe = msgs.data[5];//目標X軸
            Ye = msgs.data[6];//目標Y軸
            Theta = msgs.data[7];//目標向き
            x_error = 0;
            y_error = 0;
            t_error = 0;
            prev_t = 0;
            diff_t = 0;
            stampX = 0;
            stampY = 0;
            limit = false;
            automove = true;
            autoX = true;
            autoY = true;
            autoT = true;
            autotimer.reset();
            autotimer.start();
            break;
        case 19://PIDだけ
            Xe = msgs.data[1];
            Ye = msgs.data[2];
            Theta = msgs.data[3];
            t[0] = 0;
            t[1] = 0;
            x_error = 0;
            y_error = 0;
            t_error = 0;
            prev_t = 0;
            diff_t = 0;
            automove = true;
            autoX = true;
            autoY = true;
            autoT = true;
            stampX = 0;
            stampY = 0;
            autotimer.reset();
            autotimer.start();
            break;
        case 20://停止と補正
            safe();
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
    now.data_length = 8;
    now.data = (float *)malloc(sizeof(float)*now.data_length);
    bool xok,yok;
    int i;
    double diff[3],Pspeed[3];
    for(i = 0;i<M_NUM;i++){
        Led[i] = new DigitalOut(PIN[i][2],0);
        Moter[i][0] = new PwmOut(PIN[i][0]);
        Moter[i][1] = new PwmOut(PIN[i][1]);
        Moter[i][0]->period_us(64);//2048
        Moter[i][1]->period_us(64);//2048
    }
    for(i = 0;i < 3;i++){
        //Place[i] = new RotaryInc(RotaryPin[i][0],RotaryPin[i][1],3);
        Speed[i] = new RotaryInc(RotaryPin[i+3][0],RotaryPin[i+3][1],3);
    }
    for(i = 0;i < 4;i++){
    	Limit[i] = new DigitalIn(InPin[i],PullNone);
    }
    Limit[4] = new DigitalIn(InPin[4],PullUp);
    Limit[5] = new DigitalIn(InPin[5],PullUp);
    for(i = 0;i < 5;i++){
    	Solenoid[i] = new DigitalOut(OutPin[i],1);
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
    for(i = 0;i < 5;i++){
    	Solenoid[i]->write(0);
    }
    led.write(0);
    for(i = 0;i<3;i++){
        Speed[i]->reset();
        //Place[i]->reset();
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
        if(loop.read_ms() > 200){//10msごとに通信して通信量の調節
            now.data[0] = X;//X本来はオドメトリを送る
            now.data[1] = Y;//Y
            now.data[2] = T;//T
            now.data[3] = Yaw;
            now.data[4] = nowVx;//mm/s
            now.data[5] = nowVy;
            now.data[6] = nowVt;
            now.data[7] = automove + ((Vx || Vy || Omega) << 1) + (autoX << 2) + (autoY << 3) + (autoT << 4) + (limit << 5) + (kikou << 6);
            place.publish(&now);
            loop.reset();
        }
        Yaw *= 0.0174532925199432;//pi/180
        move();//モーターの状態を更新
        /*for(i = 0;i<3;++i){
            diff[i] = Place[i]->diff() / 256.0 / 2 * R;
            Pspeed[i] = Place[i]->getSpeed();
        }*///オドメトリ計算
        //X += -2.0/3.0*diff[0]*cos(Yaw) + 2.0/3.0*diff[1]*cos(Yaw-PI_3) + 2.0/3.0*diff[2]*cos(Yaw+PI_3);
        //Y += -2.0/3.0*diff[0]*sin(Yaw) + 2.0/3.0*diff[1]*sin(Yaw-PI_3) + 2.0/3.0*diff[2]*sin(Yaw+PI_3);
        //T +=  diff[0]*1/L3 + diff[1]*1/L3 + diff[2]*1/L3;
        //nowVx = -2.0/3.0*Pspeed[0]*cos(Yaw) + 2.0/3.0*Pspeed[1]*cos(Yaw-PI_3) + 2.0/3.0*Pspeed[2]*cos(Yaw+PI_3);
        //nowVy = -2.0/3.0*Pspeed[0]*sin(Yaw) + 2.0/3.0*Pspeed[1]*sin(Yaw-PI_3) + 2.0/3.0*Pspeed[2]*sin(Yaw+PI_3);
        //nowVt =  Pspeed[0]*1/L3 + Pspeed[1]*1/L3 + Pspeed[2]*1/L3;
        if(!kikou){
            if(drivef1 && (Limit[1]->read() || Limit[2]->read())){
            	Drive(3,0);
            	drivef1 = false;
            }else if(driveb1 && Limit[3]->read()){
            	Drive(3,0);
            	driveb1 = false;
            }
            if(drivef2 && !Limit[4]->read()){
            	Drive(4,0);
            	drivef2 = false;
            }else if(driveb2 && !Limit[5]->read()){
            	Drive(4,0);
            	driveb2 = false;
            }
        }
        if(automove){
        	if(drivebyms)drivebyms = false;
        	if(!movePID)movePID = true;
            now_t = (double)autotimer.read();
            diff_t = now_t - prev_t;
            prev_t = now_t;
            x_diff = Xe - X;
            y_diff = Ye - Y;
            t_diff = Theta - Yaw;
            if(autoX){
            	if((now_t - stampX) < t[0]){//x方向
            		Vx = Xmax/2.0*(1-cos(2.0*AMAX*(now_t - stampX)/Xmax));
            		if(xok)xok = false;
            	}else if(fabs(x_diff) < 5 && fabs(nowVx) < 30){
            		Vx = 0;
            		autoX = false;
            	}else{
            		Vx = Kp*x_diff + Ki*x_error - Kd*nowVx;
            		if(fabs(Vx) > fabs(Xmax)){
            			Vx = Xmax;
            		}else{
            			x_error += x_diff * diff_t;
            		}
            		if(!xok)xok = true;
            	}
            }else if(!xok)xok = true;

            if(autoY){
            	if((now_t - stampY)< t[1]){//y方向
                	Vy = Ymax/2.0*(1-cos(2.0*AMAX*(now_t - stampY)/Ymax));
                	if(yok)yok = false;
            	}else if((fabs(nowVy) < 30 && fabs(y_diff) < 5)){
            		if(limit && Limit[0]->read()){
            			autoY = false;
            		}else{
            			Vy = 0;
            			autoY = false;
            		}
            	}else if(limit && !Limit[0]->read()){
            		Vy = 100;
            	}else{
                	Vy = Kp*y_diff + Ki*y_error - Kd*nowVy;
                	if(fabs(Vy) > fabs(Ymax)){
                		Vy = Ymax;
                	}else{
                    	y_error += y_diff * diff_t;
                	}
            		if(!yok)yok = true;
            	}
            }else if(!yok)yok = true;

            if(autoT || limit){
                Omega = 200*t_diff + 0.2*t_error - 0.03*nowVt;
                /*if(limit){
                	Omega = Omega + Limit[0]->read() * 50 - Limit[1]->read() * 50;
                }*/
                if(Omega > 200)Omega = 200;
                else if(Omega < -200)Omega = -200;
                else t_error += t_diff * diff_t;
                if(fabs(t_diff) < PI/180 && !autoX && !autoY){
                	Omega = 0;
                	autoT = false;
                }
            }

        	if(limit){
        		if(Limit[0]->read()){
        			limit = false;
        			automove = false;
        			autoX = false;
        			autoY = false;
        			autoT = false;
        			Vx = 0;
        			Vy = 0;
        			Omega = 0;
        		}
        	}else if(!autoT && !autoX && !autoY){
            	automove = false;
            }
        }else if(kikou){
        	if(kikou & 1){//ペットボトルを取る
        		if(!Limit[1]->read() && !Limit[2]->read() && !flag){
        			Solenoid[0]->write(0);
        			Drive(3,-100);
        		}else if(!flag){
        			Drive(3,0);
        			Solenoid[0]->write(1);
        			kikoutimer.reset();
        			kikoutimer.start();
        			flag = true;
        		}else if(flag && !Limit[3]->read()){
        			if(kikoutimer.read() > 0.2){
            			Drive(3,100);
        			}
        		}else if(flag){
        			Drive(3,0);
        			kikoutimer.reset();
        			flag = false;
        			kikou &= 0xfffe;
        			kikoutimer.stop();
        			kikoutimer.reset();
        		}
        	}
        	if(kikou & 2){//ペットボトルを置く
        		if(!flag){
        			Solenoid[0]->write(0);
        			kikoutimer.reset();
        			kikoutimer.start();
        			while(kikoutimer.read() < 0.2);
        			Solenoid[1]->write(1);
        			kikoutimer.reset();
        			flag = true;
        		}else if(flag && kikoutimer.read() > 4.0){
        			Solenoid[1]->write(0);
        			flag = false;
        			kikoutimer.stop();
        			kikoutimer.reset();
            		kikou &= 0xfffd;
        		}
        	}
        	if(kikou & 4){//食事を取る,下ろす
        		if(Limit[4]->read()){
        			Drive(4,30);
        			Solenoid[2]->write(0);
        		}else{
        			Drive(4,0);
        		    kikou &= 0xfffb;
        		}
        	}
        	if(kikou & 8){//食事を取る、上げる
        		if(!flag){
        			Solenoid[2]->write(1);
        			flag = true;
        			kikoutimer.reset();
        			kikoutimer.start();
        		}else if(flag && Limit[5]->read()){
        			if(kikoutimer.read() > 0.5){
        				Drive(4,-30);
        			}
          		}else if(flag){
          			Drive(4,0);
          			kikoutimer.reset();
          			while(kikoutimer.read() < 0.5);
        		    Solenoid[2]->write(0);
        		    flag = false;
        		    kikoutimer.stop();
        		    kikoutimer.reset();
            		kikou &= 0xfff7;
        		}
        	}
        	if(kikou & 16){//食事を投げる
        		if(!flag){
        			Solenoid[4]->write(0);
        			Solenoid[3]->write(1);
        			flag = true;
        			kikoutimer.reset();
        			kikoutimer.start();
        		}else if(flag && kikoutimer.read() > 2.0){
        			Solenoid[3]->write(0);
        			Solenoid[4]->write(1);
        			flag = false;
        			kikoutimer.reset();
        			while(kikoutimer.read() < 0.5);
        			Solenoid[4]->write(0);
        			kikoutimer.stop();
        			kikoutimer.reset();
        			kikou &= 0xffef;
        		}
        	}
        	if(kikou & 32){//ペットボトル機構を前に出す
        		if(!Limit[1]->read() && !Limit[2]->read() && !flag){
        			Drive(3,-100);
        		}else if(!flag){
        			Drive(3,0);
        			kikoutimer.reset();
        			kikoutimer.start();
        			flag = true;
        		}else if(flag && kikoutimer.read() > 0.5){
        			Solenoid[0]->write(1);
        			kikoutimer.stop();
        			kikoutimer.reset();
        			flag = false;
        			kikou &= 0xffdf;
        		}
        	}
        	if(kikou & 64){//ペットボトル機構をもどす
        		if(!Limit[3]->read() && !flag){
        			Drive(3,100);
        		}else if(!flag){
        			Drive(3,0);
        			kikoutimer.reset();
        			kikoutimer.start();
        			flag = true;
        		}else if(flag && kikoutimer.read() > 0.5){
        			kikoutimer.stop();
        			kikoutimer.reset();
        			flag = false;
        			kikou &= 0xffbf;
        		}
        	}
        	if(kikou & 128){//つかむ
        		Solenoid[0]->write(1);
        		kikou &= 0xff7f;
        	}
        	if(kikou & 256){//はなす
        		Solenoid[0]->write(0);
        		kikou &= 0xfeff;
        	}
        	if(kikou & 512){
        		kikou &= 0xfdff;
        	}
        	if(kikou & 1024){
        		kikou &= 0xfbff;
        	}
        	if(kikou & 2048){
        		kikou &= 0xf7ff;
        	}
        	if(kikou & 4096){
        		kikou &= 0xefff;
        	}
        	if(kikou & 8192){
        		kikou &= 0xdfff;
        	}
        	if(kikou & 16384){
        		kikou &= 0xbfff;
        	}
        	if(kikou & 32768){
        		kikou &= 0x7fff;
        	}
        }
    }
}
