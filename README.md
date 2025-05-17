# SteeringWheelUP70  

## 项目简介  
本项目为 Up70 战队 2025 赛季舵轮代码，烧录于舵轮分控板。主控采用 STM32G431，基于 FDCAN 通信。项目存储三个舵轮信息（通过 `main.h` 中的 `define` 宏定义配置），核心控制逻辑在 CAN 接收中断中执行：  
- **CAN 1 接收中断**：处理 C620 电调和 VESC 电调的反馈信息。  
- **CAN 2 接收中断**：接收来自主机的控制请求。  

## 硬件状态指示灯说明  
舵轮配备 4 个蓝色 LED 灯，功能如下：  
- **PA3**：VESC 在线灯，亮起时表示 VESC 在线。  
- **PA2**：C620 在线灯，亮起时表示 C620 在线。  
- **PA1**：解锁运行灯，亮起时舵轮处于运行状态。  
- **PA0**：主机在线灯，亮起时舵轮与主机连接，但舵轮未运行。  

## 调零操作指南  
1. 打开 `main.h`，取消 `#define STEER_DEBUG` 的注释。  
2. 从以下代码中选择对应舵轮编号（如 `Steer_Wheel_1`）并取消注释：  
```c  
#define Steer_Wheel_1  
// #define Steer_Wheel_2  
// #define Steer_Wheel_3  
```  
3. 将代码下载至分控板，进入 Keil 的 DEBUG 模式。  
4. 给 `test_rpm_value` 赋值 `1000`，确认舵轮正方向。  
5. 将舵轮归位至正方向，查看 DEBUG 的 **Watch1** 列表中 `angle_code_raw` 参数值。  
6. 将该参数值填入 `main.c` 中对应的 `steer_zero_angle`。  
7. 重新注释 `main.h` 中的 `#define STEER_DEBUG`。  
8. 再次下载代码，完成调零。 