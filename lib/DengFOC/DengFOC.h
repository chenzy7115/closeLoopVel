//灯哥开源，遵循GNU协议，转载请著名版权！
//GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
//该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种自由软件许可协议，常用于GNU计划的软件和其他软件。
//函数声明

#ifndef DENGFOC_H
#define DENGFOC_H

#include <Arduino.h>

/*
 * ============================ DengFOC 统一接口总览 ============================
 * 本头文件是 DengFOC 库对外的“门面”，对外只暴露带 DFOC_ 前缀的函数，
 * 使用顺序通常为：
 *
 *   1) DFOC_Vbus(12);                  // 设置母线电压 + 初始化 PWM + 初始化编码器
 *   2) DFOC_alignSensor(7, 1);         // 极对数 PP、方向 DIR，并校准电角度零点
 *   3) DFOC_M0_SET_VEL_PID(P,I,D,ramp);// 设置速度环 PID（可选，不设则用默认值）
 *   4) 在 loop() 中周期调用：
 *        DFOC_M0_setVelocity(目标速度);       // 速度闭环
 *        或 DFOC_M0_set_Velocity_Angle(目标角度); // 位置闭环（外环位置 + 内环速度）
 *        或 DFOC_M0_set_Force_Angle(目标角度);    // 位置单环（纯 P 位置）
 *        或 DFOC_M0_setTorque(目标力矩电压);      // 力矩/开环电压模式
 *
 * 坐标与单位约定：
 *   - 角度：rad（但传给 PID 前会转成 度 * 180/PI，因为 PID 参数按度整定）
 *   - 速度：rad/s
 *   - 力矩：以“电压”形式表示，单位 V，范围 ±Vbus/2
 * ============================================================================
 */

//====================== 底层：SVPWM / 开环力矩 ======================
void setPwm(float Ua, float Ub, float Uc);              //!< 将三相电压钳位后写入 LEDC 占空比
float setTorque(float Uq,float angle_el);               //!< 给定 q 轴电压与电角度，执行逆 Park + 逆 Clarke 并输出 PWM
float _normalizeAngle(float angle);                     //!< 角度归一化到 [0, 2π]
void DFOC_Vbus(float power_supply);                     //!< 设置母线电压，初始化 PWM 与 AS5600 编码器
void DFOC_alignSensor(int _PP,int _DIR);                //!< 传感器与电角度对齐（校准 zero_electric_angle）
float _electricalAngle();                               //!< 根据机械角度与极对数计算当前电角度

//====================== 串口交互：接收目标值 ======================
float serial_motor_target();                            //!< 返回串口最近一次收到的目标值
String serialReceiveUserCommand();                      //!< 非阻塞读取串口命令（以 '\n' 结束）

//====================== 传感器读取（已考虑方向 DIR） ======================
float DFOC_M0_Velocity();                               //!< 读取 M0 角速度 rad/s（已低通滤波）
float DFOC_M0_Angle();                                  //!< 读取 M0 多圈角度 rad（已乘方向 DIR）

//====================== PID 参数设置与调用 ======================
void DFOC_M0_SET_ANGLE_PID(float P,float I,float D,float ramp);  //!< 设置位置环 PID
void DFOC_M0_SET_VEL_PID(float P,float I,float D,float ramp);    //!< 设置速度环 PID
float DFOC_M0_VEL_PID(float error);                     //!< 执行一次速度环 PID，输入误差（度/秒）
float DFOC_M0_ANGLE_PID(float error);                   //!< 执行一次位置环 PID，输入误差（度）

//====================== 用户简易接口（闭环模式） ======================
void DFOC_M0_set_Velocity_Angle(float Target);          //!< 位置闭环：位置环串联速度环
void DFOC_M0_setVelocity(float Target);                 //!< 速度闭环：目标角速度 rad/s
void DFOC_M0_set_Force_Angle(float Target);             //!< 位置单环：仅用位置环 PID 输出力矩
void DFOC_M0_setTorque(float Target);                   //!< 力矩开环：直接给定力矩电压 V

#endif
