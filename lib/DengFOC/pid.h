/*
 * PID 控制器类声明（SimpleFOC 风格）
 * ------------------------------------------------------------
 * 特性：
 *   - 位置式 PID，输出 = P*e + I*∫e + D*de/dt
 *   - 积分采用 Tustin（梯形）离散，比矩形积分更平稳
 *   - 输出限幅 limit + 输出变化率限幅 output_ramp（防电流冲击）
 *   - 自带采样时间 Ts 计算，只需周期性调用即可，不用手动传 dt
 */

#ifndef PID_H
#define PID_H

class PIDController
{
public:
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    //!< 重载运算符，调用方式与普通函数一致： float u = pid(error);
    float operator() (float error);

    float P; //!< 比例增益(P环增益)
    float I; //!< 积分增益（I环增益）
    float D; //!< 微分增益（D环增益）
    float output_ramp; //!< 输出变化率上限（单位/秒），0 表示不限制
    float limit; //!< 输出限幅（±limit），同时用于积分抗饱和
protected:
    float error_prev; //!< 最后的跟踪误差值（用于积分/微分）
    float output_prev;  //!< 最后一个 pid 输出值（用于斜率限幅）
    float integral_prev; //!< 最后一个积分分量值（带记忆，用于累加）
    unsigned long timestamp_prev; //!< 上次执行时间戳（us），用于算 Ts
};

#endif
