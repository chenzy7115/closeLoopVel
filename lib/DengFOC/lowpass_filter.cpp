#include "lowpass_filter.h"

/**
 * @brief 构造函数
 * @param time_constant 时间常数 Tf（单位 s），越大滤得越平滑但相位滞后越大
 */
LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant)
    , y_prev(0.0f)          // 初始输出从 0 开始
{
    timestamp_prev = micros();   // 初始化时间戳，防止首次调用 dt 异常
}


/**
 * @brief 执行一次一阶低通滤波
 * @param x 当前采样值
 * @return  滤波后的输出值
 *
 * 算法： y = alpha*y_prev + (1-alpha)*x，其中 alpha = Tf/(Tf+dt)
 * 特殊情况：
 *   - dt < 0：计时器异常，用 1ms 兜底；
 *   - dt > 0.3s：说明中间停了很久，旧状态已失效，直接跟随新值并重置状态，
 *     避免像缓慢恢复那样输出长时间的过时数据。
 */
float LowPassFilter::operator() (float x)
{
    unsigned long timestamp = micros();
    float dt = (timestamp - timestamp_prev)*1e-6f;   // us -> s

    if (dt < 0.0f ) dt = 1e-3f;          // 计时器回绕/异常：兜底 1ms
    else if(dt > 0.3f) {                 // 长时间未调用：丢失历史，直接跟随输入
        y_prev = x;
        timestamp_prev = timestamp;
        return x;
    }

    float alpha = Tf/(Tf + dt);          // 旧值权重（靠近 1 表示滤波越强）
    float y = alpha*y_prev + (1.0f - alpha)*x;   // 一阶线性加权平均
    y_prev = y;                          // 保存状态供下次使用
    timestamp_prev = timestamp;
    return y;
}
