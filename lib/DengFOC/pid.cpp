#include "pid.h"
#include <Arduino.h>
// 限幅宏：把 amt 钳位到 [low, high] 区间内
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


/**
 * @brief PID 构造函数
 * @param P     比例增益
 * @param I     积分增益
 * @param D     微分增益
 * @param ramp  输出变化率上限（防突变，0 = 不限制）
 * @param limit 输出限幅（±limit），同时用于积分抗饱和
 */
PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)
    , I(I)
    , D(D)
    , output_ramp(ramp)    // PID控制器加速度限幅
    , limit(limit)         // PID控制器输出限幅
    , error_prev(0.0f)
    , output_prev(0.0f)
    , integral_prev(0.0f)
{
    timestamp_prev = micros();   // 初始化时间戳，避免第一次调用时 Ts 过大
}

// PID 控制器函数
/**
 * @brief 执行一次 PID 运算（位置式）
 * @param error 当前误差 = 目标值 - 实际值（单位由调用者决定，本库为“度”或“度/秒”）
 * @return       PID 输出（限幅且斜率受限）
 *
 * 算法要点：
 *   - Ts 由 micros() 差值实时计算，调用者无需传 dt；
 *   - Ts 异常（<=0 或 >0.5s，多为首次调用/长时间中断）时回落为 1ms，防止突变；
 *   - I 环用 Tustin 梯形积分，比矩形积分更平滑且不易累积偏差；
 *   - integral 与 output 均受 limit 限制，实现积分抗饱和（anti-windup）。
 */
float PIDController::operator() (float error){
    // 计算两次循环中间的间隔时间
    unsigned long timestamp_now = micros();
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;   // us -> s
    // Ts 异常（首次调用/计时器回绕/长时间停顿）时用 1ms 兜底，避免微分项爆炸
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // P环
    float proportional = P * error;
    // Tustin 散点积分（I环）： integral += I * Ts * (e_now + e_prev) / 2
    float integral = integral_prev + I*Ts*0.5f*(error + error_prev);
    integral = _constrain(integral, -limit, limit);   // 积分限幅，防止积分饱和
    // D环（微分环节）：斜率 (e - e_prev)/Ts
    float derivative = D*(error - error_prev)/Ts;

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, -limit, limit);   // 总输出限幅

    if(output_ramp > 0){
        // 对PID的变化速率进行限制（类似斜率限幅，减小电流冲击/机械振动）
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp*Ts;      // 增长过快，拉回到允许斜率
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp*Ts;      // 下降过快，同理
    }
    // 保存值（为了下一次循环）
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}
