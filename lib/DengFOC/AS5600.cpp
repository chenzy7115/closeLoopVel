/*
 * AS5600 磁编码器驱动实现（SimpleFOC 精简版风格）
 * ------------------------------------------------------------
 * AS5600 是 12 位分辨率的磁旋转编码器，通过 I2C 输出绝对机械角度（单圈 0~2π）。
 * 本文件在原始单圈角度之上做了两件事：
 *   1. 通过相邻两次读数的跳变检测累计“圈数”，从而得到多圈连续角度；
 *   2. 用相邻两次角度差 / 时间差估算角速度，供电速度闭环使用。
 *
 * 角度单位约定：全部使用“弧度 rad”，单圈 0 ~ 2π，不做方向取反处理。
 */

#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h>

// 2π 常量：用 f 后缀保证是 float，避免 double 运算拖慢 ESP32 的循环速度
#define _2PI 6.28318530718f



// AS5600 相关
/**
 * @brief  读取 AS5600 的绝对机械角度（单圈角度）
 * @return 当前机械角度，单位 rad，范围 [0, 2π)
 *
 * I2C 读取流程：
 *   1. 先“写”寄存器地址 0x0C（ANGLE 高 8 位），告诉芯片接下来要从哪里读；
 *   2. 再“读”2 个字节，得到 12 位角度原始值（高 8 位 + 低 4 位）。
 * 注意：0x0C 与 0x0D 组成 12 位角度寄存器，MSB 在高地址的低 4 位上，
 * 所以不能简单地把两个字节拼成 16 位，必须按位掩码截取。
 */
double Sensor_AS5600::getSensorAngle() {
  uint8_t angle_reg_msb = 0x0C;   // ANGLE 寄存器高字节地址（0x0C~0x0D）

  byte readArray[2];              // 存放读回的两个字节：[0]=高字节 0x0C，[1]=低字节 0x0D
  uint16_t readValue = 0;         // 拼接还原后的 12 位原始角度值

  // ---- 第一步：写入要读取的寄存器地址 ----
  wire->beginTransmission(0x36);  // 0x36 是 AS5600 的固定 I2C 从机地址
  wire->write(angle_reg_msb);     // 指定从 0x0C 寄存器开始读
  wire->endTransmission(false);   // false = 发送 STOP 前不释放总线（重复起始），保证是“写地址+读数据”的原子操作


  // ---- 第二步：连续读取 2 个字节 ----
  wire->requestFrom(0x36, (uint8_t)2);
  for (byte i=0; i < 2; i++) {
    readArray[i] = wire->read();
  }

  // ---- 第三步：按 12 位分辨率拆解位域 ----
  // AS5600 的 12 位角度值定义为：
  //   0x0C 寄存器 bit3~bit0 -> 角度 bit11~bit8（共 4 位）
  //   0x0D 寄存器 bit7~bit0 -> 角度 bit7 ~bit0（共 8 位）
  // 因此高字节只用到低 4 位，低字节 8 位全用。
  int _bit_resolution=12;         // 编码器分辨率：12 bit
  int _bits_used_msb=11-7;        // 高字节中有效的位数 = 11-7 = 4 位（bit11~bit8）
  float cpr = pow(2, _bit_resolution);   // 每圈计数 = 2^12 = 4096
  int lsb_used = _bit_resolution - _bits_used_msb;  // 低字节承载的位数 = 12-4 = 8 位

  uint8_t lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );        // 低字节掩码 = 0xFF（保留全部 8 位）
  uint8_t msb_mask = (uint8_t)( (2 << _bits_used_msb) - 1 );  // 高字节掩码 = 0x0F（只保留低 4 位）

  // 低位直接取；高位掩码后左移 8 位（lsb_used）再相加，拼成完整 12 位数值
  readValue = ( readArray[1] &  lsb_mask );
  readValue += ( ( readArray[0] & msb_mask ) << lsb_used );
  // 原始计数换算成弧度： readValue / 4096 * 2π
  return (readValue/ (float)cpr) * _2PI;
}

//AS5600 相关

//=========角度处理相关=============
/**
 * @brief 构造函数
 * @param Mot_Num 电机编号（预留给多电机场景，便于统一在本文件中索引）
 */
Sensor_AS5600::Sensor_AS5600(int Mot_Num) {
   _Mot_Num=Mot_Num;  //使得 Mot_Num 可以统一在该文件调用

}

/**
 * @brief 编码器初始化
 * @param _wire 使用的 I2C 总线（默认 &Wire，可传入 &Wire1 走另一组引脚）
 *
 * 初始化时会“丢弃式”读取几次角度，目的是：
 *   - 等待 AS5600 上电稳定（芯片上电后需要一定时间才能输出有效数据）；
 *   - 把 angle_prev / vel_angle_prev 等历史值填充为真实值，
 *     防止第一次调用 getVelocity() 时因初值为 0 而算出巨大的假速度。
 */
void Sensor_AS5600::Sensor_init(TwoWire* _wire) {
    wire=_wire;
    wire->begin();   //电机Sensor
    delay(500);      // 等待 AS5600 上电与磁铁检测稳定
    getSensorAngle();          // 丢弃第 1 次读数（可能无效/抖动）
    delayMicroseconds(1);
    vel_angle_prev = getSensorAngle();   // 记录速度计算的“上一次角度”
    vel_angle_prev_ts = micros();        // 记录速度计算的“上一次时间戳”
    delay(1);
    getSensorAngle();          // 丢弃一次读数，拉开 1ms 时间间隔
    delayMicroseconds(1);
    angle_prev = getSensorAngle();       // 记录角度计算的“上一次角度”
    angle_prev_ts = micros();            // 记录角度计算的时间戳
}

/**
 * @brief 周期调用刷新角度状态（不返回角度，仅更新内部缓存）
 *
 * 每次调用会：
 *   1. 读取新的单圈机械角度 val；
 *   2. 与上一次角度比较，若跳变超过 0.8 圈（约 4.8rad），
 *      说明越过了 0/2π 边界，据此对圈数 full_rotations 做 ±1 修正。
 *   判断阈值取 0.8*2π 而不是 0.5*2π，是为了给高速运行留出余量，
 *   避免单次采样间隔内转过半圈以上时被误判。
 */
void Sensor_AS5600::Sensor_update() {
    float val = getSensorAngle();        // 新的单圈机械角度
    angle_prev_ts = micros();            // 先打时间戳（越接近采样时刻越准）
    float d_angle = val - angle_prev;    // 与上次角度的差值，用于判断是否跨圈
    // 圈数检测
    // d_angle > 0.8圈  => 实际是从 2π 附近“回绕”到 0 附近（正向转过零点），圈数应 -1
    // d_angle < -0.8圈 => 实际是从 0 附近“回绕”到 2π 附近（反向转过零点），圈数应 +1
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1;
    angle_prev = val;                    // 更新缓存，供下次比较
}

/**
 * @brief 获取机械角度（单圈，已去圈数）
 * @return 角度，单位 rad，范围 [0, 2π)
 */
float Sensor_AS5600::getMechanicalAngle() {
    return angle_prev;
}

/**
 * @brief 获取连续多圈角度（含圈数信息）
 * @return 角度，单位 rad，可超出 [0, 2π)，正反转连续变化
 */
float Sensor_AS5600::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;   // 圈数*2π + 当前单圈角度
}

/**
 * @brief 获取角速度（差分法估算）
 * @return 角速度，单位 rad/s；方向与角度增加方向一致
 *
 * 注意：该函数有“副作用”，每次调用会把当前角度/时间戳存为新的参考点，
 * 因此应保证在固定周期（如控制循环）内调用一次，否则测速的分母会不同。
 */
float Sensor_AS5600::getVelocity() {
    // 计算采样时间（micros 差值 转 秒）
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    // 快速修复奇怪的情况（微溢出）：时间戳相同或回绕时留一个保守的 1ms，避免除零得到 inf
    if(Ts <= 0) Ts = 1e-3f;
    // 速度计算：本次与上次之间的“总角度差”（圈数差*2π + 单圈角差）除以时间
    float vel = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;
    // 保存变量以待将来使用（作为下次速度计算的参考点）
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}
