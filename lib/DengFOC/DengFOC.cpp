/*
 * DengFOC 核心实现（ESP32 + AS5600 有感 FOC）
 * ------------------------------------------------------------
 * 本文件提供整条 FOC 控制链：
 *   AS5600 角度 -> 电角度 -> 力矩电压 Uq -> 逆 Park -> 逆 Clarke -> SVPWM/PWM
 * 以及周边：母线电压/引脚/编码器初始化、电角度零点校准、
 * 速度环/位置环 PID、串口接收目标值。
 *
 * 注意：完整初始化请调用 DFOC_Vbus()（它会开关硬件外设），
 *       不要单独调用内部函数。
 */

#include <Arduino.h>
#include "AS5600.h"
#include "lowpass_filter.h"
#include "pid.h"

// 限幅宏：把 amt 钳位到 [low, high] 区间内（用于电压/占空比保护）
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// 调试打印开关：0=关闭（默认）。
// 初始化/校准信息属于非数字的文本帧，会混入 VOFA+ 的 FireWater 数据流导致波形出错，
// 因此默认关闭；不用上位机、只看串口日志时可改为 1 打开。
#define DFOC_DEBUG_SERIAL 0
float voltage_power_supply;              //!< 母线电压 V（由 DFOC_Vbus() 设置，各环节的归一化基准）
float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;     //!< Park/Clarke 变换中间量：αβ 轴电压、三相电压
#define _3PI_2 4.71238898038f            //!< 3π/2，校准电角度时使用的固定电角度
float zero_electric_angle=0;             //!< 电角度零点偏移（由 DFOC_alignSensor() 校准）
int PP=1,DIR=1;                          //!< 电机极对数 / 旋转方向（1 或 -1）
int pwmA = 32;                           //!< A 相 PWM 引脚
int pwmB = 33;                           //!< B 相 PWM 引脚
int pwmC = 25;                           //!< C 相 PWM 引脚

//低通滤波初始化
LowPassFilter M0_Vel_Flt = LowPassFilter(0.01); // Tf = 10ms   //M0速度环
//PID
// 构造参数：(P, I, D, output_ramp, limit)
// 限幅选 voltage_power_supply/2 是因为三相输出以 Vbus/2 为中心偏移
PIDController vel_loop_M0 = PIDController(2, 0, 0, 100000, voltage_power_supply/2);
PIDController angle_loop_M0 = PIDController(2, 0, 0, 100000, 100);

//AS5600
Sensor_AS5600 S0=Sensor_AS5600(0);       //!< M0 编码器实例（编号 0）
TwoWire S0_I2C = TwoWire(0);             //!< 独立的 I2C 外设（I2C0），不占用默认 Wire，引脚可自定义

//=================PID 设置函数=================
//速度PID
/**
 * @brief 设置 M0 速度环 PID 参数
 * @param P,I,D  三环增益
 * @param ramp   输出变化率上限（防突变）
 */
void DFOC_M0_SET_VEL_PID(float P,float I,float D,float ramp)   //M0速度环PID设置
{
  vel_loop_M0.P=P;
  vel_loop_M0.I=I;
  vel_loop_M0.D=D;
  vel_loop_M0.output_ramp=ramp;
}
//角度PID
/**
 * @brief 设置 M0 位置环 PID 参数
 * @param P,I,D  三环增益
 * @param ramp   输出变化率上限（防突变）
 */
void DFOC_M0_SET_ANGLE_PID(float P,float I,float D,float ramp)   //M0角度环PID设置
{
  angle_loop_M0.P=P;
  angle_loop_M0.I=I;
  angle_loop_M0.D=D;
  angle_loop_M0.output_ramp=ramp;
}


//M0速度PID接口
/**
 * @brief 执行一次 M0 速度环 PID
 * @param error 速度误差（单位与 PID 参数整定单位一致，本库为 度/秒）
 * @return PID 输出（作为力矩电压指令）
 */
float DFOC_M0_VEL_PID(float error)   //M0速度环
{
   return vel_loop_M0(error);

}
//M0角度PID接口
/**
 * @brief 执行一次 M0 位置环 PID
 * @param error 位置误差，单位：度（调用处已 *180/PI）
 * @return PID 输出（作为速度指令，再送入速度环）
 */
float DFOC_M0_ANGLE_PID(float error)
{
  return angle_loop_M0(error);
}


// 归一化角度到 [0,2PI]
/**
 * @brief 把任意角度归一化到 [0, 2π)
 * @param angle 输入角度（可能为负或超出 2π）
 * @return 归一化后的角度，范围 [0, 2π)
 */
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);
  //三目运算符。格式为condition ? expr1 : expr2
  //其中，condition 是要求值的条件，条件成立时返回 expr1，否则返回 expr2。
  //fmod 函数的余数符号与除数相同；负数角度取余后加 2*PI，即可归一化到 [0, 2PI]。
}


// 设置PWM到控制器输出
/**
 * @brief 把三相电压写入 PWM 通道
 * @param Ua,Ub,Uc 三相输出相电压（V），相对地为参考，范围 [0, Vbus]
 *
 * 由于普通三相逆变桥无法输出负电压，SVPWM 会把相电压抬升 Vbus/2（见 setTorque），
 * 因此这里只需把电压映射为 0~1 的占空比即可。
 */
void setPwm(float Ua, float Ub, float Uc) {
  // 限制上限
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);
  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  //Arduino-ESP32 3.x 的 LEDC 写入接口使用 GPIO 引脚
  // 8 位精度，取值 0~255
  ledcWrite(pwmA, dc_a*255);
  ledcWrite(pwmB, dc_b*255);
  ledcWrite(pwmC, dc_c*255);
}

/**
 * @brief FOC 核心：给定 q 轴电压与电角度，输出三相 PWM
 * @param Uq       q 轴电压指令（等效力矩），单位 V，内部钳位到 ±Vbus/2
 * @param angle_el 当前电角度（rad）
 * @return         固定返回 0（预留接口）
 *
 * 计算链：
 *   逆 Park：  Ualpha = -Uq*sinθ，Ubeta = Uq*cosθ（d 轴电压 Ud 设为 0）
 *   逆 Clarke：Ua = Ualpha
 *               Ub = (√3*Ubeta - Ualpha)/2
 *               Uc = (-Ualpha - √3*Ubeta)/2
 *   偏置：   三项均 + Vbus/2，把交流量抬到正电压区间
 */
float setTorque(float Uq,float angle_el) {
  S0.Sensor_update(); //更新传感器数值（保证角度尽量新）
  Uq=_constrain(Uq,-(voltage_power_supply)/2,(voltage_power_supply)/2);
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换（Inverse Park）
  Ualpha =  -Uq*sin(angle_el);
  Ubeta =   Uq*cos(angle_el);

  // 克拉克逆变换（Inverse Clarke）并抬升到正电压区间
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
  return 0.0f;
}

/**
 * @brief 总线电压设置 + 硬件初始化（整个库的初始化入口）
 * @param power_supply 母线电压 V（建议填入实测值，直接影响力矩标定）
 *
 * 依次完成：
 *   1. 记录母线电压；
 *   2. 配置三相 GPIO 为输出，用 LEDC 绑定通道/30kHz/8bit；
 *   3. 初始化 I2C0（SDA=19, SCL=18, 400kHz）并初始化 AS5600；
 *   4. 重建速度环 PID（因为初始化前 voltage_power_supply 还是 0，用作限幅会出现 0 限幅）
 */
void DFOC_Vbus(float power_supply)
{
  voltage_power_supply=power_supply;
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  //Arduino-ESP32 3.x 使用 ledcAttachChannel 绑定 GPIO、频率、精度和通道
  ledcAttachChannel(pwmA, 30000, 8, 0);  //PWM频道0，频率30kHz，精度8位
  ledcAttachChannel(pwmB, 30000, 8, 1);  //PWM频道1，频率30kHz，精度8位
  ledcAttachChannel(pwmC, 30000, 8, 2);  //PWM频道2，频率30kHz，精度8位
#if DFOC_DEBUG_SERIAL
  Serial.println("完成PWM初始化设置");   // 文本帧：会干扰 VOFA+ 波形，默认不输出
#endif

  //AS5600
  S0_I2C.begin(19,18, 400000UL);   // SDA=GPIO19, SCL=GPIO18, 400kHz（快速模式）
  S0.Sensor_init(&S0_I2C);   //初始化编码器0
#if DFOC_DEBUG_SERIAL
  Serial.println("编码器加载完毕");     // 文本帧：会干扰 VOFA+ 波形，默认不输出
#endif

  //PID 加载（用真实母线电压重建限幅，避免上面全局构造时 Vbus=0 导致限幅为 0）
  vel_loop_M0 = PIDController(2, 0, 0, 100000, voltage_power_supply/2);
 }


/**
 * @brief 计算当前电角度
 * @return 电角度，范围 [0, 2π) = 机械角度 * 极对数 PP * 方向 DIR - 零点偏移
 */
float _electricalAngle(){
  return  _normalizeAngle((float)(DIR *  PP) * S0.getMechanicalAngle()-zero_electric_angle);
}


/**
 * @brief 电角度/传感器对齐（校准 zero_electric_angle）
 * @param _PP  电机极对数
 * @param _DIR 旋转方向（1 或 -1）
 *
 * 原理：给一个固定电角度（_3PI_2）通入小力矩，转子会被强拉到对应电角度位置，
 *       等 1s 稳定后读回机械角度算电角度，即得到该位置下的零点偏移 zero_electric_angle。
 *       要求：校准时电机必须能自由转动，且校准后 PP/DIR 不能再改。
 */
void DFOC_alignSensor(int _PP,int _DIR)
{
  PP=_PP;
  DIR=_DIR;
  setTorque(3, _3PI_2);  //起劲：给 3V 的 q 轴电压，把转子拉到固定电角度
  delay(1000);           //等转子稳定
  S0.Sensor_update();  //更新角度，方便下面电角度读取
  zero_electric_angle=_electricalAngle();   // 记录该位置对应的电角度作为零点
  setTorque(0, _3PI_2);  //松劲（解除校准）
#if DFOC_DEBUG_SERIAL
  Serial.print("0电角度：");Serial.println(zero_electric_angle);   // 文本帧：会干扰 VOFA+ 波形，默认不输出
#endif
}

/**
 * @brief 读取 M0 多圈角度
 * @return 角度 rad（已乘方向 DIR）
 */
float DFOC_M0_Angle()
{
  return DIR*S0.getAngle();
}

//有滤波
/**
 * @brief 读取 M0 角速度（带低通滤波）
 * @return 角速度 rad/s（已乘方向 DIR）
 */
float DFOC_M0_Velocity()
{
  //获取速度数据并滤波
  float vel_M0_ori=S0.getVelocity();                 // 原始差分速度
  float vel_M0_flit=M0_Vel_Flt(DIR*vel_M0_ori);      // 先乘方向再低通滤波
  return vel_M0_flit;   //考虑方向
}

//==============串口接收==============
float motor_target;      //!< 串口收到的目标值（速度或角度，由上层决定）
int commaPosition;       //!< 换行符位置（字符串拆分用）
/**
 * @brief 非阻塞读取串口命令
 * @return 本次接收到的完整命令字符串（未接收完时返回空串）
 *
 * 协议：以 '\n' 作为一帧结束符，帧内内容可被 toDouble() 解析为数字，
 *       解析结果存到 motor_target，可通过 serial_motor_target() 获取。
 *       配合上层用 while(Serial.available()) 持续轮询即可。
 */
String serialReceiveUserCommand() {

  // 保存接收到的字符
  static String received_chars;   // 静态变量：跨调用保留未完成的半帧数据

  String command = "";

  while (Serial.available()) {
    // 读取新字节
    char inChar = (char)Serial.read();
    // 添加到字符串缓冲区
    received_chars += inChar;

    // 用户输入结束（收到换行符表示一帧结束）
    if (inChar == '\n') {

      // 执行用户命令
      command = received_chars;

      commaPosition = command.indexOf('\n');
      if(commaPosition != -1)
      {
          // 取换行符前的子串并转数字（兼容 "12.5\n" 这类格式）
          motor_target = command.substring(0,commaPosition).toDouble();            //电机目标速度
          // 注意：这里不能回显 motor_target，否则会把单个数字混进 VOFA+ 的逗号分隔数据帧中，
          // 导致波形出现错位帧（通道数时多时少）。
      }
      // 重置命令缓冲区，准备接收下一帧
      received_chars = "";
    }
  }
  return command;
}


/**
 * @brief 获取串口最近一次解析出的目标值
 * @return motor_target（单位由上层约定：速度 rad/s 或 角度 rad）
 */
float serial_motor_target()
{
  return motor_target;
}



//================简易接口函数================
/**
 * @brief 位置闭环（外环位置 + 内环速度，即串级 PID）
 * @param Target 目标角度 rad
 *
 * 内层 (Target - Angle) 仅作为位置误差传入位置环，
 * 位置环输出被当作速度目标送入速度环，最后转成力矩电压。
 * 注：两组误差都乘 180/PI 转成“度”，因为 PID 参数按度整定。
 */
void DFOC_M0_set_Velocity_Angle(float Target)
{
 setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI)),_electricalAngle());   //角度闭环
}

/**
 * @brief 速度闭环
 * @param Target 目标角速度 rad/s
 */
void DFOC_M0_setVelocity(float Target)
{
  setTorque(DFOC_M0_VEL_PID((Target-DFOC_M0_Velocity())*180/PI),_electricalAngle());   //速度闭环
}

/**
 * @brief 位置单环（仅位置环 PID 直接输出力矩）
 * @param Target 目标角度 rad
 *
 * 相比串级模式响应更快但稳定性差，适合轻负载、单向定位场景。
 */
void DFOC_M0_set_Force_Angle(float Target)   //力位
{
  setTorque(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI),_electricalAngle());
}

/**
 * @brief 力矩开环（调试用）
 * @param Target 力矩电压 V，范围 ±Vbus/2
 */
void DFOC_M0_setTorque(float Target)
{
  setTorque(Target,_electricalAngle());
}
