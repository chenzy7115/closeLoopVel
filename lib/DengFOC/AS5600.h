/*
 * AS5600 磁编码器驱动类声明
 * ------------------------------------------------------------
 * 该类把 AS5600 原始的“单圈绝对角度”包装成 SimpleFOC 风格的传感器接口：
 *   Sensor_init()       -> 初始化 I2C 并预填历史状态
 *   Sensor_update()     -> 周期调用，刷新角度并累计圈数（必须每个控制周期调用）
 *   getMechanicalAngle()-> 单圈角度 [0, 2π)
 *   getAngle()          -> 多圈连续角度（含圈数）
 *   getVelocity()       -> 由相邻两次角度差估算的角速度 rad/s
 */

#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include "Wire.h"

class Sensor_AS5600
{
  public:
    Sensor_AS5600(int Mot_Num);                        //!< 构造函数，传入电机编号
    void Sensor_init(TwoWire* _wire = &Wire);          //!< 初始化：绑定 I2C 总线、预读并填充历史状态
    void Sensor_update();                              //!< 刷新角度并检测圈数（需周期性调用）
    float getAngle();                                  //!< 获取多圈连续角度 rad
    float getVelocity();                               //!< 获取角速度 rad/s（差分法）
    float getMechanicalAngle();                        //!< 获取单圈机械角度 rad
    double getSensorAngle();                           //!< 直接读取 I2C 原始角度，单位 rad
  private:
    int _Mot_Num;               //!< 电机编号（多电机时用于区分实例）
    //AS5600 变量定义
    //int sensor_direction=1;       //编码器旋转方向定义（预留：如需反转角度方向在此约定）
    float angle_prev=0;         //!< 最后一次调用 getSensorAngle() 的输出结果，用于得到完整的圈数和速度
    long angle_prev_ts=0;       //!< 上次调用 getAngle 的时间戳（单位：us）
    float vel_angle_prev=0;     //!< 最后一次调用 getVelocity 时的角度（测速参考点）
    long vel_angle_prev_ts=0;   //!< 最后速度计算时间戳（单位：us）
    int32_t full_rotations=0;   //!< 总圈数计数（正转/反转过零点时 ±1）
    int32_t vel_full_rotations=0; //!< 用于速度计算的先前完整旋转圈数（测速参考点）
    TwoWire* wire;              //!< 绑定的 I2C 总线指针（默认 &Wire，可传 &Wire1）
};

#endif
