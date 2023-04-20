#ifndef __COMMUNICAT_FUNCTION_H__
#define __COMMUNICAT_FUNCTION_H__

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <ros/ros.h>

extern "C"{
    #include "motor_communicate/motor_function.h"
}

//這邊主要是設定一個抽象的輪子物件，方便之後管理每一個輪子
//每個function都是在對特別的暫存器進行寫入或讀取
class wheel{
    public:

    wheel(uint8_t address);

    //由於我們當時是用分檔位去控制速度的，所以下面這個區塊在將我們調教好的數值儲存起來
    void settingRpmData(int *xdata, int xlength, int *ydata, int ylength);
    void settingRpmBias(int *xdata, int xlength, int *ydata, int ylength);
    void settingRpmRotation(int speed);
    void settingXPID(float data_load[3], float data_unload[3]);
    void settingYPID(float data_load[3], float data_unload[3]);
    void settingRotationPID(float *data_load, float *data_unload);

    //PID 只在變化移動方向的時候，去把對應方向的PID寫入控制器，其他三個是移動指令
    //這邊主要是對0x43，也就是速度控制器去寫入數值來進行速度控制
    //如果要使用位置控制就要同時對0x46和0x47兩個控制器進行寫入
    //如果要向l298n那樣類似輸出pwm訊號控制的話就對0x42進行寫入
    void set_X_Speed(int speed, int bias);
    void set_Y_Speed(int speed, int bias);
    void setRoatation(int direction);
    void setPID(int direction, int speed, bool load);

    //stop是有煞車的停止，另外一個是自由停止，但我覺得差別不大
    //對0x40暫存器寫入數值可以停止
    void stop();
    void freeStop();

    //獲得馬達轉速
    void getRpm();

    //回傳輪子的轉速和設定轉速
    int output_rpm();
    int output_target();

    private:

    //把Data容器清空
    void clearMsg();

    //宣告一個 data容器
    serialData msg;

    //儲存此輪子的控制器address
    uint8_t controller_Address;

    int target_speed;
    int last_target_speed;
    int current_rpm;

    int *rpm_X_Data;
    int rpm_X_Data_Length;

    int *rpm_Y_Data;
    int rpm_Y_Data_Length;

    int *rpm_X_Bias;
    int rpm_X_Bias_Length;

    int *rpm_Y_Bias;
    int rpm_Y_Bias_Length;

    int rpm_Rotation;

    float *x_Pid_data_load;
    float *y_Pid_data_load;
    float *rotation_Pid_data_load;

    float *x_Pid_data_unload;
    float *y_Pid_data_unload;
    float *rotation_Pid_data_unload;
};

#endif