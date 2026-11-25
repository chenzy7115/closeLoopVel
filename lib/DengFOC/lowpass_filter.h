#include <Arduino.h>

#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

/**
  * 低通滤波器类（一阶 RC 数字低通，SimpleFOC 风格）
  * ------------------------------------------------------------
  * 传递函数： y = (Tf*y_prev + dt*x) / (Tf + dt)
  * 其中 Tf 为时间常数（越大滤波越强、相位滞后越大），dt 为实际采样间隔。
  * 用途：用于速度环之前的测速平滑，抑制 AS5600 差分测速的量化噪声。
  * 使用：在固定周期内调用 flt(x)，会自动按实际 dt 计算系数，
  *       因此采样周期变化时也能保持等效截止频率。
  */

class LowPassFilter
{
public:
    /**
     * @Tf - 低通滤波时间常数（单位 s，例如 0.01 = 10ms）
     */
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    //!< 重载运算符：传入原始信号，返回滤波后信号
    float operator() (float x);
    float Tf; //!< 低通滤波时间常数

protected:
    unsigned long timestamp_prev;  //!< 最后执行时间戳（us），用于计算 dt
    float y_prev; //!< 上一个循环中的过滤后的值（滤波器状态记忆）
};

#endif
