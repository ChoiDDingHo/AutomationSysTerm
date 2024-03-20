#include "mbed.h"
#include "Thread.h"
#include "rtos.h" 
#include <cstdint>
#include "FastPWM.h"
#include "MPU9250.h"
#include "PMW3901.h"
#include "GP2A.h"

#define MCU_CONTROL_RATE 6 // 전체 제어주기

RawSerial pc(USBTX, USBRX, 115200);

//////////////////////// Thread ////////////////////////////
Thread Print_thread(osPriorityHigh); //osPriorityAboveNormal
Thread MPU_thread(osPriorityRealtime); //osPriorityRealtime
Thread Psd_thread(osPriorityAboveNormal); //osPriorityHigh


//////////////////////// Timer /////////////////////////////
Timer PPM_timer; // ppm엔코더에 대해 타이머 지정
Timer IMU_steady_timer; // IMU 안정화를 위해 안정화될 시간을 측정
Timer I2C_timer; // I2C 통신이 timeout되지 않는지 확인하는 용도
Timer Print_timer; // Print 또한 thread로 빼서 이에 대해 타이머 지정
Timer UWB_timer;


//////////////////////// Motor /////////////////////////////
#define MIN_PULSE_LENGTH 1000 //ESC가 받는 최소 PWM
#define MAX_PULSE_LENGTH 2000 //ESC가 받는 최대 PWM

FastPWM motorA(PB_5); // 노초보회 (모터 순서 신호 선 색)
FastPWM motorB(PB_4);
FastPWM motorC(PB_10);
FastPWM motorD(PA_8);


//////////////////////////PRINT//////////////////////////////
#define PRINT_RATE 2000 // Print 제어주기 100ms


////////////////////////// PPM //////////////////////////////
#define max_high_time 1500 // ppm엔코더에서 받아오는 최대 값
#define min_high_time 700 // ppm엔코더에서 받아노는 최소 값
#define InterruptPin PA_10 // ppm엔코더의 핀 위치치 (D2)

int throttleA = 0, throttleB = 0, throttleC = 0, throttleD = 0; // 각 모터에 따른 스로틀 값
volatile float throttle; 
volatile float filltered_throttle;

InterruptIn PPM(InterruptPin); // 인터럽트 감지 함수
int PPM_ch[8]; // ppm 엔코더 체널 List
uint32_t PPM_all[19]; // ppm엔코더에 값 대입하기 위한 List

char PPM_INDEX=0, PPM_init=0; // ppm엔코더 값 변수

int Gear; // Gear가 0단인지 1단인지 구분하기 위한 변수 생성


//////////////////////// MPU9250 ////////////////////////////
#define IMU_UPDATE_RATE 6 // IMU센서 제어주기

MPU9250 mpu9250; // MPU9250 Class 변수 명 mpu9250으로 선정

DigitalIn sdaDummy(PB_9, PullUp); // 복잡한 회로에 의한 noise를 방지하기 위해 nucleo 보드 안에 내장되어있는 pullup저항 활성화 코드
DigitalIn sclDummy(PB_8, PullUp);

float tmp_angle_x, tmp_angle_y, tmp_angle_z;
float filltered_angle_x, filltered_angle_y, filltered_angle_z;
bool imu_steady = false;
bool imu_i2c_timeout = false;

volatile float tmp_gx;
volatile float tmp_gy;


////////////////////////// PID /////////////////////////////
double dt=0;

volatile float Roll_target_angle = 0;
float Roll_angle_input;
float Roll_rate_input;
float Roll_output = 0;
float Roll_angle_kp = 2.0; //2.0 //6.8  //1.0
float Roll_angle_ki = 0;
float Roll_rate_kp = 1.0; //1.0 //0.25  //1.4
float Roll_rate_ki = 0;
float Roll_rate_kd = 0.35; //0.3 // 0.35  //0.6
float Roll_angle_iterm;
float Roll_rate_iterm;
float Roll_rate_dterm;

volatile float Pitch_target_angle = 0;
float Pitch_angle_input;
float Pitch_rate_input;
float Pitch_output = 0;
float Pitch_angle_kp = 2.4; //2.4 //6  //0.8
float Pitch_angle_ki = 0;
float Pitch_rate_kp = 0.9; //0.9 //0.3  //1.13
float Pitch_rate_ki = 0;
float Pitch_rate_kd = 0.35; //0.35 // 0.35  //0.6
float Pitch_angle_iterm;
float Pitch_rate_iterm;
float Pitch_rate_dterm;

volatile float Yaw_target_rate = 0.0;
volatile float Yaw_target_angle_Rx = 0.0;
volatile float Yaw_prev_dterm;
volatile float Yaw_prev;
float Yaw_angle_input;
float Yaw_rate_input;
float Yaw_output = 0;
float Yaw_rate_kp = 8; //8
float Yaw_rate_ki = 0.0;
float Yaw_rate_kd = 0.01;
float Yaw_angle_iterm;
float Yaw_rate_pterm;
float Yaw_rate_iterm;
float Yaw_rate_dterm;


///////////////////// Optical Flow //////////////////////
#define Opticla_Update_Rate 100

PMW3901 pmw3901(PB_15,PB_14,PB_13,PB_12);

volatile float x_dot, y_dot;
float tmp_x_dot = 0, tmp_y_dot = 0;
float befor_x_dot, befor_y_dot;
float Nx = 300;
float Ny = 300;
char theta = 42;

volatile float prev_dot_x_dterm;
volatile float prev_x_dot;
volatile float Roll_target_angle_Rx = 0;
volatile float Filltered_Roll_target_angle_Rx = 0;
float input_x_dot;
float kp_x_dot = 12;
float ki_x_dot;
float kd_x_dot = 3;
volatile float pterm_x_dot, iterm_x_dot, dterm_x_dot;
volatile float output_x_dot;
volatile float filltered_output_x_dot;

volatile float prev_dot_y_dterm;
volatile float prev_y_dot;
volatile float Pitch_target_angle_Rx = 0;
volatile float Filltered_Pitch_target_angle_Rx = 0;
float input_y_dot;
float kp_y_dot = 5;
float ki_y_dot;
float kd_y_dot = 1;
volatile float pterm_y_dot, iterm_y_dot, dterm_y_dot;
volatile float output_y_dot;
volatile float filltered_output_y_dot;

volatile float tmp_px;
volatile float tmp_py;


////////////////////////// PSD ///////////////////////////
#define Psd_Update_Rate 6

GP2A psd(PB_1, 10, 80, 24, 0);

volatile float height;
float psd_val;
float filltered_psd;


//////////////////////// UWB, BLT /////////////////////////
#define UWB_UPDATE_RATE 500 // IMU센서 제어주기

Serial uwb(PA_11, PA_12, 115200);
Serial ble(PA_9, PB_7, 115200);

char buf[256];
int buf_size = 0;


//-------------------------------------------------------------------------------------
void print_thread_loop(){
    
    uint64_t Now_p,Work_p;

//////////////////////////// UWB, BLT //////////////////////////////
    while(true)
    {
        Now_p=rtos::Kernel::get_ms_count();

        

        if (uwb.readable()) {
            pc.printf("dddd \n");
            char c = uwb.getc();
            if (c == ':'){
                c = '#';
            }
            if (c == 'm'){
                c = '#';
            }
            if (c == '#') {
                    if (buf_size > 0) { // 시작 문자(#)가 아닌 경우
                        buf[buf_size] = '\0'; // 버퍼의 마지막에 널 문자를 추가하여 문자열 종료
                        if (buf_size==4){
                            pc.printf("%s\n", buf); // 받은 문자열을 출력
                            ble.printf("%s\n", buf);
                        }
                        // 파이썬 루프백 코드 작성 (buf 배열을 파이썬으로 전송하는 방법은 제외)
                    }
                    buf_size = 0;
                } else {
                    buf[buf_size] = c;
                    buf_size++;
                }
        }


        // pc.printf("[GEAR] : %d ", PPM_ch[3]);
        // pc.printf("\t");
        // pc.printf("[ROLL] : %d ", Roll_target_angle_Rx);
        // pc.printf("\t");
        // pc.printf("[THROTTLE] : %d ", throttle);
        // pc.printf("\t");
        // pc.printf("[PPM_THROTTLE] : %d ", PPM_ch[5]);
        // pc.printf("\t");
        // pc.printf("\n");
        // pc.printf("[YAW] : %d ", Yaw_target_rate);
        // pc.printf("\t");
        // pc.printf("[PITCH] : %d ", Pitch_target_angle_Rx);
        // pc.printf("\t");
        // pc.printf("[Fil_THROTTLE] : %f ", filltered_throttle);
        // pc.printf("\t");
        // pc.printf("\n");
        // pc.printf("[X] : %f ", filltered_angle_x);
        // pc.printf("\t");
        // pc.printf("[Y] : %f ", filltered_angle_y);
        // pc.printf("\t");
        // pc.printf("\n");
        // pc.printf("t: %u", imu_i2c_timeout);
        // pc.printf("\t");
        // pc.printf("[Z] : %f ", filltered_angle_z);
        // pc.printf("\t");
        // pc.printf("PID : %f,%f ", Roll_output, Pitch_output);
        // pc.printf("%f", output_y_dot);
        // pc.printf("\t");
        // pc.printf("\n");

        // pc.printf("H: %f ", height);
        // pc.printf("\t");
        // pc.printf("X_dot: %f ", x_dot);
        // pc.printf("\t");
        // pc.printf("Y_dot: %f ", y_dot);
        // pc.printf("\t");
        // pc.printf("X: %f ", pmw3901.px);
        // pc.printf("\t");
        // pc.printf("Y: %f ", pmw3901.py);
        // pc.printf("\t");
        // pc.printf("GX: %f ", gx);
        // pc.printf("\t");
        // pc.printf("GY: %f ", gy);
        // pc.printf("\t");
        // pc.printf("\n");

        // pc.printf("%f", dt);
        // pc.printf("\n");

        // pc.printf("%f", filltered_throttle);
        // pc.printf("\n");

        // pc.printf("%d ", PPM_ch[5]);
        // pc.printf("\t");
        // pc.printf("   %f ", filltered_angle_x);
        // pc.printf("\t");
        // pc.printf("   %f ", filltered_angle_y);
        // pc.printf("\t");
        // pc.printf("   %f,%f ", Roll_output, Pitch_output);
        // pc.printf("\n");
        // pc.printf("   %f,%f ", Roll_target_angle, Pitch_target_angle);
        // pc.printf("\n");
        // pc.printf("   %f,%f ", output_x_dot, output_y_dot);
        // pc.printf("\n");
        // pc.printf("   %f,%f ", x_dot, y_dot);
        // pc.printf("\n");
        // pc.printf("   %f,%f ", Roll_target_angle_Rx, Pitch_target_angle_Rx);
        // pc.printf("\n");


        
        

        Work_p=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count() + PRINT_RATE-(Work_p-Now_p));
    }
}


//-------------------------------------------------------------------------------------
double mapping(float x, float b, float c, float d, float e)
{
  double k = ((d-e)/(b-c)); // k는 [d,e]범위로 맵핑하기 위한 스케일링 요소를 계산함
  double result = k*(x-c)+e;

  return result;
}
// 스케일을 변환해주는 함수


//-------------------------------------------------------------------------------------
float constrain(float x, float x_min, float x_max){
    return (x>x_max)? x_max: ((x<x_min)? x_min:x);
}
// 경계 구분 함수 (값이 Max, Min을 넘지 못하도록 함)


//-------------------------------------------------------------------------------------
void MAPPING_CH() // ppm 엔코더를 통해서 ESC가 인식할 수 있는 범위로 맵핑해주는 함수
{
    for(int i=0;i<8;i++)
    {
        if(PPM_ch[i]<min_high_time)
        {
            PPM_ch[i]=min_high_time;
        }
        if(PPM_ch[i]>max_high_time && PPM_ch[i]<1600)
        {
            PPM_ch[i]=max_high_time;
        }
        if(abs(PPM_ch[i])>1600)
        {
            PPM_ch[i]=710;
        }
    }

    if(PPM_ch[3] <= 900)
    {
        Gear = 0;
    }    
    else if(PPM_ch[3] > 900 && PPM_ch[3] < 1510) 
    {
        Gear = 1;
    }
    else 
    {
        Gear = 0;
    }

    //////////CH4 ROLL /////////////
    Roll_target_angle_Rx = mapping(PPM_ch[6], 705.0, 1500.0, -7.0, 7.0);
    // Roll_target_angle_Rx = mapping(PPM_ch[6], 705.0, 1500.0, -5.0, 5.0);

    Filltered_Roll_target_angle_Rx = 0.8 * Filltered_Roll_target_angle_Rx + 0.2 * Roll_target_angle_Rx;

    ///////////CH5 Throttle///////
    if (PPM_ch[5] < 1490)
    {
        throttle = mapping(PPM_ch[5], 705, 1500, 1000, 2000);
        filltered_throttle = 0.96 * filltered_throttle + 0.04 * throttle;
    }
    else if (abs(PPM_ch[5]) > 1490)
    {
        throttle = 0.0;
        filltered_throttle = 0.0;
    }
    


    ////////////CH6 YAW//////////
    Yaw_target_rate = mapping(PPM_ch[4], 705.0, 1500.0, -7.0, 7.0);
    // Yaw_target_rate = mapping(PPM_ch[4], 705.0, 1500.0, -5.0, 5.0);


    ///////////CH7 PITCH//////////////
    Pitch_target_angle_Rx = mapping(PPM_ch[7], 705.0, 1500.0, -7.0, 7.0);
    // Pitch_target_angle_Rx = mapping(PPM_ch[7], 705.0, 1500.0, -5.0, 5.0);

     Filltered_Pitch_target_angle_Rx = 0.8 * Filltered_Pitch_target_angle_Rx + 0.2 * Pitch_target_angle_Rx;


}


//-------------------------------------------------------------------------------------
void RISING()
{
    PPM_timer.reset();
}


//-------------------------------------------------------------------------------------
void FALLING()
{
    PPM_all[PPM_INDEX] = PPM_timer.read_us();
    PPM_INDEX++;

    if(PPM_INDEX==17)
    {
        for(int i=18;i>-1;i--)
        {
            if(PPM_all[i]>6000)
            {
                PPM_init=i;
            }
        }
        for(int i=0;i<8;i++)
        {
            PPM_ch[i]=PPM_all[PPM_init + i + 1];
        }
        PPM_INDEX = 0;
        MAPPING_CH();
    }
}


//-------------------------------------------------------------------------------------
void setup_mpu9250()
{
    mpu9250.resetMPU9250();

    mpu9250.MPU9250SelfTest(SelfTest);  

    mpu9250.calibrateMPU9250(gyroBias, accelBias); 

    mpu9250.initMPU9250();

    mpu9250.getAres();

    mpu9250.getGres();
}


//-------------------------------------------------------------------------------------
void IMU_thread_loop()
{
    uint64_t Now_time = 0;
    uint64_t Work_time = 0;
    bool warning1_imu = true;
    bool warning2_imu = true;

    IMU_steady_timer.start();

    while(1)
    {
        Now_time=rtos::Kernel::get_ms_count();
        mpu9250.read_data();

        if(IMU_steady_timer.read_ms()>=6000)
            {
                imu_steady=true;
                IMU_steady_timer.stop();
                break;
            }

        Work_time=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count() + IMU_UPDATE_RATE-(Work_time-Now_time));
    }    
        
    I2C_timer.start();

    while(1)
    {
        I2C_timer.reset();
        Now_time=rtos::Kernel::get_ms_count();

        int MPU_I2C_time1=I2C_timer.read_us();
          
        if (warning1_imu || warning2_imu)
        {
            mpu9250.read_data();
        }
        else
        {
            imu_i2c_timeout = true;
        }
        
        int MPU_I2C_time2=I2C_timer.read_us();

        if(MPU_I2C_time2-MPU_I2C_time1>=50000) // I2C통신이 50ms을 넘기면 I2C통신 중단
        {
            if(warning1_imu == true)
            {
                warning1_imu = false;
            }
            else
            {
                warning2_imu = false;
            }
        }

        float alpha = 0.90;

        roll = atan2((ay) , sqrt(ax * ax + az * az)) * (180.0 / PI);
        pitch = atan2((-ax) , sqrt(ay * ay + az * az)) * (180.0 / PI);

        tmp_angle_x = filltered_angle_x + gx * deltat;
        tmp_angle_y = filltered_angle_y + gy * deltat;
        tmp_angle_z = filltered_angle_z + gz * deltat;

        filltered_angle_x = alpha * tmp_angle_x + (1.0-alpha) * roll;
        filltered_angle_y = alpha * tmp_angle_y + (1.0-alpha) * pitch;
        filltered_angle_z = tmp_angle_z;

        tmp_gx = gx;
        tmp_gy = gy;
        
        Work_time=rtos::Kernel::get_ms_count(); 
        ThisThread::sleep_until(rtos::Kernel::get_ms_count() + IMU_UPDATE_RATE-(Work_time-Now_time));  
    }
}


//-----------------------------------------------------------------------------------
void Psd_thread_loop()
{
    int32_t Now_psd,Work_psd;
    while(true)
    {
        Now_psd = rtos::Kernel::get_ms_count();
        psd_val = psd.getDistance();

        filltered_psd = 0.96 * filltered_psd + 0.04 * psd_val;

        height = filltered_psd;
        Work_psd = rtos::Kernel::get_ms_count();

        ThisThread::sleep_until(rtos::Kernel::get_ms_count() + Psd_Update_Rate - (Work_psd - Now_psd));
    }
}


//-----------------------------------------------------------------------------------
void PID_term(volatile float& prev_dterm, volatile float& prev, float target,
float input, float kp, float ki, float kd, volatile float &pterm, 
volatile float &iterm, volatile float &dterm, volatile float &output)
{
    volatile float error;
    volatile float prev_error;

    error = target - input; // 오차 = 목표값 - 입력값
    prev_error = target - prev; // 이전 오차 = 목표값 - 이전 값

    pterm = error * kp; // 비례항
    iterm += error * ki * dt; // 적분항
    dterm = ((error - prev_error)/dt) * kd; // 미분항

    dterm = prev_dterm*0.9 + dterm*0.1; // LPF

    output = pterm + iterm + dterm;

    prev_dterm = dterm;
    prev = input;
}


//------------------------------------------------------------------------------------
void dualPID(float target_angle,
             float angle_input,
             float rate_input,
             float angle_kp,
             float angle_ki,
             float rate_kp,
             float rate_ki,
             float rate_kd,
             float &angle_iterm,
             float &rate_iterm,
             float &rate_dterm,
             float &output)
{
    float angle_error;
    float target_rate;
    float rate_error;
    float prev_error;
    float angle_pterm, rate_pterm;
    float rate_prev_dterm;
    float prev_rate;
    
    angle_error = target_angle - angle_input;

    angle_pterm = angle_kp * angle_error;

    angle_iterm += angle_ki * angle_error * dt;
    
    target_rate = angle_pterm; //외부 루프 각도 P제어, 내부 루프 각속도 제어의 target rate가 된다.

    rate_error = target_rate - rate_input;
    prev_error = target_rate - prev_rate; // 이전 오차 = 목표값 - 이전 값
    
    rate_pterm = rate_kp * rate_error; 
    rate_iterm += rate_ki * rate_error * dt; 
    rate_dterm = ((rate_error - prev_error)/dt) * rate_kd;

    rate_dterm = 0.9 * rate_prev_dterm + 0.1 * rate_dterm;

    if(angle_iterm > 50)
    {
        angle_iterm = 50;
    }
    else if(angle_iterm < -50)
    {
        angle_iterm = -50;
    }

    if(rate_iterm > 30)
    {
        rate_iterm = 30;
    }
    else if(rate_iterm < -30)
    {
        rate_iterm = -30;
    }

    output = rate_pterm + rate_iterm + angle_iterm + rate_dterm;

    prev_rate = rate_input;
    rate_prev_dterm = rate_dterm;
}


//------------------------------------------------------------------------------------
void calcYPRtoDualPID()
{
    Roll_angle_input = filltered_angle_x;
    Roll_rate_input = gx;

    dualPID(Roll_target_angle,
            Roll_angle_input,
            Roll_rate_input,
            Roll_angle_kp,
            Roll_angle_ki,
            Roll_rate_kp,
            Roll_rate_ki,
            Roll_rate_kd,
            Roll_angle_iterm,
            Roll_rate_iterm,
            Roll_rate_dterm,
            Roll_output
    );

    if(Roll_output > 100){
        Roll_output = 100;
    }
    else if(Roll_output < -100){
        Roll_output = -100;
    }

    Pitch_angle_input = filltered_angle_y;
    Pitch_rate_input = gy;

    dualPID(Pitch_target_angle,
            Pitch_angle_input,
            Pitch_rate_input,
            Pitch_angle_kp,
            Pitch_angle_ki,
            Pitch_rate_kp,
            Pitch_rate_ki,
            Pitch_rate_kd,
            Pitch_angle_iterm,
            Pitch_rate_iterm,
            Pitch_rate_dterm,
            Pitch_output
    );

    if(Pitch_output > 200){
        Pitch_output = 200;
    }
    else if(Pitch_output < -200){
        Pitch_output = -200;
    }

    Yaw_rate_input = gz;

    PID_term(Yaw_prev_dterm, Yaw_prev, Yaw_target_rate, Yaw_rate_input, Yaw_rate_kp, Yaw_rate_ki, 
    Yaw_rate_kd, Yaw_rate_pterm, Yaw_rate_iterm, Yaw_rate_dterm, Yaw_output);

    if(Yaw_output > 50)
    {
        Yaw_output = 50;
    }
    else if(Yaw_output < -50)
    {
        Yaw_output = -50;
    }
}


//------------------------------------------------------------------------------------
void target_pid()
{
    PID_term(prev_dot_x_dterm, prev_x_dot, Filltered_Roll_target_angle_Rx, input_x_dot, kp_x_dot, ki_x_dot, kd_x_dot,
    pterm_x_dot, iterm_x_dot, dterm_x_dot, output_x_dot);


    if (output_x_dot >= 4)
    {
        output_x_dot = 4;
    }
    else if (output_x_dot <= -15)
    {
        output_x_dot = -15;
    }

    if(abs(filltered_output_x_dot - output_x_dot)>=15)
    {
        output_x_dot = filltered_output_x_dot;
    }
    filltered_output_x_dot = output_x_dot;

    PID_term(prev_dot_y_dterm, prev_y_dot, Filltered_Pitch_target_angle_Rx, input_y_dot, kp_y_dot, ki_y_dot, kd_y_dot,
    pterm_y_dot, iterm_y_dot, dterm_y_dot, output_y_dot);


    if (output_y_dot >= 15)
    {
        output_y_dot = 15;
    }
    else if (output_y_dot <= -15)
    {
        output_y_dot = -15;
    }

    if(abs(filltered_output_y_dot - output_y_dot)>=15)
    {
        output_y_dot = filltered_output_y_dot;
    }
    filltered_output_y_dot = output_y_dot;
}

//-----------------------------------------------------------------------------------
void optical_psd_velocity()
{
    tmp_px = pmw3901.px;
    tmp_py = pmw3901.py;

    x_dot = height * theta * tmp_py / (6 * Nx) * 1.5 - (height * tmp_gx) / 450.; //450
    y_dot = height * theta * tmp_px / (6 * Nx) * 1.5 + (height * tmp_gy) / 500.; //500

    tmp_x_dot = x_dot;
    tmp_y_dot = y_dot;

    x_dot = 0.9 * befor_x_dot + 0.1 * x_dot;
    y_dot = 0.9 * befor_y_dot + 0.1 * y_dot;

    if(x_dot >= 4)
    {
        x_dot = befor_x_dot;
    }
    else if (x_dot < -4)
    {
        x_dot = befor_x_dot;
    }

    if(y_dot >= 4)
    {
        y_dot = befor_y_dot;
    }
    else if (y_dot < -4)
    {
        y_dot = befor_y_dot;
    }

    input_x_dot = x_dot;
    input_y_dot = y_dot;

    befor_x_dot = x_dot;
    befor_y_dot = y_dot;
}

//------------------------------------------------------------------------------------
void init_sensor()
{
    pmw3901.init(); // Optical flow
    setup_mpu9250(); // IMU
}

//-------------------------------------------------------------------------------------
int main()
{
    // motorA.pulsewidth_us(2000); // ESC 켈브
    // motorB.pulsewidth_us(2000);
    // motorC.pulsewidth_us(2000);
    // motorD.pulsewidth_us(2000);
    // wait_ms(15000);

    // motorA.pulsewidth_us(1000);
    // motorB.pulsewidth_us(1000);
    // motorC.pulsewidth_us(1000);
    // motorD.pulsewidth_us(1000);
    // wait_ms(10000);

    motorA.pulsewidth_us(1000);
    motorB.pulsewidth_us(1000);
    motorC.pulsewidth_us(1000);
    motorD.pulsewidth_us(1000);
    wait_ms(2000);

    uwb.printf("AT+anchor_tag=0\r\n");
    wait_ms(5000);
    uwb.printf("AT+RST\r\n");
    wait_ms(5000);
    uwb.printf("AT+interval=5\r\n");
    wait_ms(5000);
    uwb.printf("AT+switchdis=1\r\n");
    wait_ms(5000);

    init_sensor();

    osThreadSetPriority(osThreadGetId(),osPriorityRealtime7);

    PPM.enable_irq(); // 인터럽트 활성화 함수
    PPM_timer.start();

    PPM.rise(&RISING);
    PPM.fall(&FALLING);

    MPU_thread.start(&IMU_thread_loop);
    Print_thread.start(&print_thread_loop);
    Psd_thread.start(&Psd_thread_loop);

    uint64_t lastUpdate = 0, firstUpdate = 0, Now = 0, Work=0;

    while(true) 
    {
        Now=rtos::Kernel::get_ms_count();

        if(imu_steady)
        { 
            break;
        }

        Work=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count() + MCU_CONTROL_RATE-(Work-Now));
    }
    
        while(true)
        {
            Now=rtos::Kernel::get_ms_count();

            Roll_target_angle = Filltered_Roll_target_angle_Rx;
            Pitch_target_angle = Filltered_Pitch_target_angle_Rx;

            pmw3901.read();

            optical_psd_velocity(); // 선속도 제어


        if(Gear == 0)
        {
            Roll_output = 0;
            Pitch_output = 0;
            Yaw_output = 0;

            throttleA = 1000;
            throttleB = 1000;
            throttleC = 1000;
            throttleD = 1000;
        }
        else if(Gear == 1)
        {
            dt = (rtos::Kernel::get_ms_count() - Work)/1000.;

            target_pid(); // Devo10으로 Roll, Pitch, Yaw값 타겟팅

            calcYPRtoDualPID(); // PID 제어
            
            throttleA = filltered_throttle - Roll_output + Pitch_output - Yaw_output + output_x_dot + output_y_dot + 5;
            throttleB = filltered_throttle - Roll_output - Pitch_output + Yaw_output + output_x_dot - output_y_dot;
            throttleC = filltered_throttle + Roll_output + Pitch_output + Yaw_output - output_x_dot + output_y_dot + 6; // 15
            throttleD = filltered_throttle + Roll_output - Pitch_output - Yaw_output - output_x_dot - output_y_dot; // 12

            throttleA = constrain(throttleA,1000,2000);
            throttleB = constrain(throttleB,1000,2000);
            throttleC = constrain(throttleC,1000,2000);
            throttleD = constrain(throttleD,1000,2000);
        }

        motorA.pulsewidth_us(throttleA);
        motorB.pulsewidth_us(throttleB);
        motorC.pulsewidth_us(throttleC);
        motorD.pulsewidth_us(throttleD);

        Work=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count() + MCU_CONTROL_RATE-(Work-Now));
    }
}
