#include <Arduino.h>
#include "AS5600.h"
#include "lowpass_filter.h"
#include "pid.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float voltage_power_supply;
float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;
#define _3PI_2 4.71238898038f
float zero_electric_angle=0;
int PP=1,DIR=1;
int pwmA = 32;
int pwmB = 33;
int pwmC = 25;

//低通滤波初始化
LowPassFilter M0_Vel_Flt = LowPassFilter(0.01); // Tf = 10ms   //M0速度环
//PID
PIDController vel_loop_M0 = PIDController(2, 0, 0, 100000, voltage_power_supply/2);
PIDController angle_loop_M0 = PIDController(2, 0, 0, 100000, 100);

//AS5600
Sensor_AS5600 S0=Sensor_AS5600(0);
TwoWire S0_I2C = TwoWire(0);

//=================PID 设置函数=================
//速度PID
void DFOC_M0_SET_VEL_PID(float P,float I,float D,float ramp)   //M0速度环PID设置
{
  vel_loop_M0.P=P;
  vel_loop_M0.I=I;
  vel_loop_M0.D=D;
  vel_loop_M0.output_ramp=ramp;
}
//角度PID
void DFOC_M0_SET_ANGLE_PID(float P,float I,float D,float ramp)   //M0角度环PID设置
{
  angle_loop_M0.P=P;
  angle_loop_M0.I=I;
  angle_loop_M0.D=D;
  angle_loop_M0.output_ramp=ramp;
}


//M0速度PID接口
float DFOC_M0_VEL_PID(float error)   //M0速度环
{
   return vel_loop_M0(error);

}
//M0角度PID接口
float DFOC_M0_ANGLE_PID(float error)
{
  return angle_loop_M0(error);
}


// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);
  //三目运算符。格式为condition ? expr1 : expr2
  //其中，condition 是要求值的条件，条件成立时返回 expr1，否则返回 expr2。
  //fmod 函数的余数符号与除数相同；负数角度取余后加 2*PI，即可归一化到 [0, 2PI]。
}


// 设置PWM到控制器输出
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
  ledcWrite(pwmA, dc_a*255);
  ledcWrite(pwmB, dc_b*255);
  ledcWrite(pwmC, dc_c*255);
}

float setTorque(float Uq,float angle_el) {
  S0.Sensor_update(); //更新传感器数值
  Uq=_constrain(Uq,-(voltage_power_supply)/2,(voltage_power_supply)/2);
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换
  Ualpha =  -Uq*sin(angle_el);
  Ubeta =   Uq*cos(angle_el);

  // 克拉克逆变换
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
  return 0.0f;
}

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
  Serial.println("完成PWM初始化设置");

  //AS5600
  S0_I2C.begin(19,18, 400000UL);
  S0.Sensor_init(&S0_I2C);   //初始化编码器0
  Serial.println("编码器加载完毕");

  //PID 加载
  vel_loop_M0 = PIDController(2, 0, 0, 100000, voltage_power_supply/2);
 }


float _electricalAngle(){
  return  _normalizeAngle((float)(DIR *  PP) * S0.getMechanicalAngle()-zero_electric_angle);
}


void DFOC_alignSensor(int _PP,int _DIR)
{
  PP=_PP;
  DIR=_DIR;
  setTorque(3, _3PI_2);  //起劲
  delay(1000);
  S0.Sensor_update();  //更新角度，方便下面电角度读取
  zero_electric_angle=_electricalAngle();
  setTorque(0, _3PI_2);  //松劲（解除校准）
  Serial.print("0电角度：");Serial.println(zero_electric_angle);
}

float DFOC_M0_Angle()
{
  return DIR*S0.getAngle();
}

//有滤波
float DFOC_M0_Velocity()
{
  //获取速度数据并滤波
  float vel_M0_ori=S0.getVelocity();
  float vel_M0_flit=M0_Vel_Flt(DIR*vel_M0_ori);
  return vel_M0_flit;   //考虑方向
}

//==============串口接收==============
float motor_target;
int commaPosition;
String serialReceiveUserCommand() {

  // 保存接收到的字符
  static String received_chars;

  String command = "";

  while (Serial.available()) {
    // 读取新字节
    char inChar = (char)Serial.read();
    // 添加到字符串缓冲区
    received_chars += inChar;

    // 用户输入结束
    if (inChar == '\n') {

      // 执行用户命令
      command = received_chars;

      commaPosition = command.indexOf('\n');
      if(commaPosition != -1)
      {
          motor_target = command.substring(0,commaPosition).toDouble();            //电机目标速度
          Serial.println(motor_target);
      }
      // 重置命令缓冲区
      received_chars = "";
    }
  }
  return command;
}


float serial_motor_target()
{
  return motor_target;
}



//================简易接口函数================
void DFOC_M0_set_Velocity_Angle(float Target)
{
 setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI)),_electricalAngle());   //角度闭环
}

void DFOC_M0_setVelocity(float Target)
{
  setTorque(DFOC_M0_VEL_PID((Target-DFOC_M0_Velocity())*180/PI),_electricalAngle());   //速度闭环
}

void DFOC_M0_set_Force_Angle(float Target)   //力位
{
  setTorque(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI),_electricalAngle());
}

void DFOC_M0_setTorque(float Target)
{
  setTorque(Target,_electricalAngle());
}
