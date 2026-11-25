#include <Arduino.h>
#include "DengFOC.h"

// DengFOC V0.2
// 灯哥开源，遵循GNU协议，转载请著名版权！
// GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
// 该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的一种自由软件许可协议，常用于GNU计划的软件和其他软件。

int Sensor_DIR = 1; // 传感器方向
int Motor_PP = 7;   // 电机极对数

void setup()
{
  Serial.begin(115200);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH); // V4电机使能

  DFOC_Vbus(12.6); // 设定驱动器供电电压
  DFOC_alignSensor(Motor_PP, Sensor_DIR);
}

void loop()
{
  // 设置速度环PID
  DFOC_M0_SET_VEL_PID(0.005, 0.1, 0, 0);
  // 接收串口目标速度
  serialReceiveUserCommand();
  // 设置速度并执行速度闭环
  DFOC_M0_setVelocity(serial_motor_target());

  // ===== VOFA+ 波形输出（FireWater 协议）=====
  // 数据格式："ch0,ch1,...,\n"，在 VOFA+ 中选择 "FireWater" 协议、波特率 115200
  // 通道顺序：0 = 目标速度(rad/s)   1 = 实际速度(rad/s)
  const unsigned long PRINT_INTERVAL_MS = 20;   // 20ms 一帧 => 50Hz 波形
  static unsigned long last_print_ms = 0;
  if (millis() - last_print_ms >= PRINT_INTERVAL_MS)
  {
    last_print_ms = millis();
    // 只能输出纯数字和英文逗号，不要带中文/单位，否则 VOFA+ 会解析失败
    Serial.printf("%.3f,%.3f\n", serial_motor_target(), DFOC_M0_Velocity());
  }

  // // 下面代码是没有加低通滤波，速度控制，直接使用传感器速度，电机会明显的抖动，还会被外界环境干扰
  // float Sensor_Vel = DFOC_M0_Velocity();
  // float Kp = 0.01;
  // setTorque(Kp * ((serial_motor_target() - Sensor_Vel) * 180 / PI), _electricalAngle());
  // // //接收串口目标速度
  // serialReceiveUserCommand();
}