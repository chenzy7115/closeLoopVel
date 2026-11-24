# ESP32 闭环速度 FOC 示例工程

本项目基于 PlatformIO 和 DengFOC V0.2 示例移植，面向 WEMOS LOLIN32 Lite，实现 **AS5600 有感闭环速度 FOC**。电机位置和速度由 AS5600 磁编码器反馈，速度环 PID 输出 \(U_q\)，经 FOC 坐标变换后生成三相 PWM 占空比。

本项目采用电压矢量控制，**不包含电流采样、电流环或过流保护**，适合用于算法学习、低速平台调试和后续硬件保护功能的扩展。

## 主要功能

- AS5600 磁编码器角度读取与多圈计数
- 开机零电角度校准
- 机械角速度计算与低通滤波
- 速度环 PID 控制
- 帕克逆变换和克拉克逆变换
- 三相 30 kHz、8 位精度 PWM 输出
- 串口接收目标速度，波特率为 115200
- 保留 DengFOC 示例原有库名、类名、函数名和接口参数

## 当前默认参数

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| 开发板 | `lolin32_lite` | WEMOS LOLIN32 Lite，ESP32 |
| 框架 | Arduino | 使用 Arduino-ESP32 |
| 电机极对数 `Motor_PP` | `7` | 必须与实际电机参数一致 |
| 传感器方向 `Sensor_DIR` | `1` | 根据编码器安装方向调整 |
| 母线电压 | `12.6 V` | 同时用于 PWM 占空比和电压限幅计算 |
| 速度环 PID | `P=0.005, I=0, D=0, ramp=0` | 在 `src/main.cpp` 中设置 |
| 速度低通时间常数 | `0.01 s` | 即 10 ms |
| PWM A/B/C | GPIO `32/33/25` | 对应 LEDC 通道 `0/1/2` |
| PWM 频率/精度 | `30000 Hz / 8 bit` | 占空比范围为 `0~255` |
| I²C SDA/SCL | GPIO `19/18` | AS5600 地址为 `0x36` |
| I²C 时钟 | `400 kHz` | 使用 ESP32 的 `TwoWire(0)` |
| 电机使能 | GPIO `12` | 上电后输出高电平 |
| 串口监视波特率 | `115200` | 与上传波特率相互独立 |

> 编码器方向、电机极对数和相序必须与实际硬件匹配。若方向相反，可调整 `Sensor_DIR`；若电机无法正常换相，应检查极对数、编码器角度和三相相序。

## 项目结构

```text
.
├── src/
│   └── main.cpp                 # 主程序入口
├── lib/
│   └── DengFOC/
│       ├── DengFOC.h            # DengFOC 公开接口
│       ├── DengFOC.cpp          # FOC、校准、速度环和串口实现
│       ├── AS5600.h             # AS5600 类声明
│       ├── AS5600.cpp           # 编码器读取、多圈计数和速度计算
│       ├── pid.h                # PIDController 类声明
│       ├── pid.cpp              # PID 实现
│       ├── lowpass_filter.h     # LowPassFilter 类声明
│       ├── lowpass_filter.cpp   # 一阶低通滤波实现
│       └── library.json         # PlatformIO 本地库元数据
├── platformio.ini               # PlatformIO 环境配置
├── include/                     # 项目头文件目录，当前未使用
├── lib/README                   # PlatformIO 库目录说明
├── test/                        # PlatformIO 测试目录，当前未使用
└── .pio/                        # 构建产物，由 PlatformIO 生成
```

DengFOC 作为本地库放入 `lib/DengFOC`，PlatformIO 会根据 `#include "DengFOC.h"` 自动编译和链接，不需要额外的 `lib_deps`。`Wire` 来自 Arduino-ESP32 框架。

## 控制流程

### 上电初始化

`setup()` 依次完成以下操作：

1. 初始化串口，波特率为 115200。
2. 将 GPIO `12` 设置为电机使能输出并拉高。
3. 调用 `DFOC_Vbus(12.6)`：
   - 保存母线电压；
   - 初始化三相 PWM；
   - 初始化 AS5600 所在的 I²C 总线；
   - 重新初始化速度 PID 的输出限幅。
4. 调用 `DFOC_alignSensor(Motor_PP, Sensor_DIR)`：
   - 施加一个较小的定位力矩；
   - 等待 1 秒；
   - 记录当前电角度作为零电角度；
   - 撤销定位力矩；
   - 通过串口打印校准结果。

电角度计算为：

\[
\theta_e = \operatorname{normalize}\left(DIR \times PP \times \theta_m - \theta_{e0}\right)
\]

其中，\(\theta_m\) 为编码器机械角，\(\theta_{e0}\) 为零电角度，\(\operatorname{normalize}\) 将角度归一化到 \([0, 2\pi)\)。

### 速度闭环

`loop()` 中重复执行：

1. 设置速度环 PID 参数。
2. 接收串口目标速度。
3. 调用 `DFOC_M0_setVelocity()` 执行一次速度闭环。

速度误差由目标速度与滤波后的反馈速度相减得到，并转换为角度制数值参与 PID 计算：

\[
e_\omega = \left(\omega_{target} - \omega_{filtered}\right)\frac{180}{\pi}
\]

PID 输出经过母线电压一半的幅值限制后作为 \(U_q\)。FOC 坐标变换为：

\[
U_\alpha = -U_q\sin(\theta_e), \qquad U_\beta = U_q\cos(\theta_e)
\]

\[
U_a = U_\alpha + \frac{V_{bus}}{2}
\]

\[
U_b = \frac{\sqrt{3}U_\beta - U_\alpha}{2} + \frac{V_{bus}}{2}
\]

\[
U_c = \frac{-\sqrt{3}U_\beta - U_\alpha}{2} + \frac{V_{bus}}{2}
\]

最后根据三相电压计算占空比并写入 PWM。

## 串口使用

1. 烧录程序后打开串口监视器。
2. 设置波特率为 115200。
3. 输入目标速度并回车换行，例如输入 `0.5`。
4. 程序会打印接收到的目标速度，并将其应用到速度环。

目标速度沿用参考示例的单位，即 **rad/s**。正负值分别表示正转和反转，但实际旋转方向还取决于 `Sensor_DIR`、电机接线和相序。

启动时全局目标速度初始化为 0。请先从小速度开始调试，不要直接输入较大的目标值。

## 构建、上传和监视

需要安装 PlatformIO，并确保能够下载 `espressif32` 平台及 Arduino-ESP32 框架。

在项目根目录的 PlatformIO 终端中执行：

```bash
pio run -e lolin32_lite
```

编译成功后上传：

```bash
pio run -e lolin32_lite -t upload
```

打开串口监视器：

```bash
pio device monitor -e lolin32_lite
```

`platformio.ini` 中已设置：

```ini
monitor_speed = 115200
```

该配置只控制串口监视速度，不会改变固件上传波特率。

### 已验证环境

- PlatformIO Core 6.2.0
- Espressif 32 平台 8.0.0
- Arduino-ESP32 3.3.12
- 开发板：WEMOS LOLIN32 Lite

该环境已通过完整编译和链接。由于 `platformio.ini` 未锁定平台版本，后续平台升级后仍建议重新执行构建验证。

## PID 调整

主程序使用以下速度环参数：

```cpp
DFOC_M0_SET_VEL_PID(0.005, 0.00, 0, 0);
```

| 参数 | 作用 | 调整建议 |
| --- | --- | --- |
| `P` | 比例增益 | 增大可加快响应，过大会引起振荡 |
| `I` | 积分增益 | 用于减小稳态误差，过大会积累积分并造成超调 |
| `D` | 微分增益 | 用于抑制变化，编码器噪声下可能放大抖动 |
| `ramp` | PID 输出变化率限制 | 设为 0 表示不启用输出斜率限制 |

速度环输出限幅为母线电压的一半。当前默认 12.6 V 母线对应约 6.3 V 的 \(U_q\) 幅值限制。

## 主要接口

### 初始化与底层控制

| 接口 | 说明 |
| --- | --- |
| `DFOC_Vbus(float power_supply)` | 设置母线电压并初始化 PWM、I²C 和速度 PID |
| `DFOC_alignSensor(int _PP, int _DIR)` | 设置极对数和方向并校准零电角度 |
| `setPwm(float Ua, float Ub, float Uc)` | 输出三相 PWM 占空比 |
| `setTorque(float Uq, float angle_el)` | 根据 \(U_q\) 和电角度执行 FOC 输出 |
| `_normalizeAngle(float angle)` | 角度归一化 |
| `_electricalAngle()` | 获取当前电角度 |

### 反馈与闭环接口

| 接口 | 说明 |
| --- | --- |
| `DFOC_M0_Angle()` | 获取累计机械角 |
| `DFOC_M0_Velocity()` | 获取滤波后的速度反馈 |
| `DFOC_M0_setVelocity(float Target)` | 速度闭环 |
| `DFOC_M0_set_Velocity_Angle(float Target)` | 角度环与速度环串级闭环 |
| `DFOC_M0_set_Force_Angle(float Target)` | 角度闭环 |
| `DFOC_M0_setTorque(float Target)` | 直接输出指定 \(U_q\) |

### PID 与串口接口

| 接口 | 说明 |
| --- | --- |
| `DFOC_M0_SET_VEL_PID(float P, float I, float D, float ramp)` | 设置速度 PID |
| `DFOC_M0_SET_ANGLE_PID(float P, float I, float D, float ramp)` | 设置角度 PID |
| `DFOC_M0_VEL_PID(float error)` | 执行速度 PID |
| `DFOC_M0_ANGLE_PID(float error)` | 执行角度 PID |
| `serialReceiveUserCommand()` | 接收串口命令 |
| `serial_motor_target()` | 获取当前串口目标速度 |

## 相对参考示例的适配

为适配当前 PlatformIO 和 Arduino-ESP32 3.x 环境，进行了以下调整：

- 将旧式 `ledcSetup`、`ledcAttachPin` 替换为 `ledcAttachChannel`。
- PWM 写入由通道号改为对应 GPIO 引脚。
- 将 PID 的指定初始化器改为调用原有 `PIDController` 构造函数。
- 修正 `setTorque` 头文件声明与实现返回类型不一致的问题，保留原头文件的 `float` 接口。
- `DFOC_M0_setVelocity(Target)` 使用传入的 `Target`，不再忽略参数。
- 主循环先接收串口命令，再执行速度闭环。
- 为头文件增加包含保护，并添加本地库元数据。

上述调整未重命名原示例的库、类或公开函数。

## 安全注意事项

- 上电校准会施加定位力矩，电机可能转动，务必确保负载端可自由运动或已固定。
- 调试时先降低母线电压，并使用小速度目标。
- 本项目没有电流环、过流保护、堵转保护、看门狗或急停功能，不能直接用于有安全风险的实际设备。
- 确认功率级驱动管、母线电压、电流能力和电机参数匹配后再上电。
- 编码器未连接或读取失败时，当前程序没有完整的故障处理，可能输出错误控制量。
- 不建议将手靠近旋转部件，也不要在带负载情况下直接进行零电角度校准。

## 来源与许可

本项目以 DengFOC V0.2 的 `DengFOC_Lib_Lesson7_CloseLoop_Vel` 示例为主要参考，保留源代码中的作者和 GNU GPL 声明。参考代码声明遵循 GNU General Public License；分发、修改或衍生使用时，请遵守相应 GPL 条款并保留版权及许可声明。

当前工程已完成编译验证，但尚未在实际电机和功率级硬件上完成运行验证。
