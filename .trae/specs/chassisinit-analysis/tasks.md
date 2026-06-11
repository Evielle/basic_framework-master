# Tasks

- [x] Task 1: 分析 ChassisInit 执行流程
  - 梳理从电机初始化到通信方式初始化的完整流程

- [x] Task 2: 分析所有变量和结构体
  - 静态全局变量
  - 电机配置结构体
  - PID配置结构体
  - CAN通信配置

- [x] Task 3: 分析调用的函数
  - PowerControlInit(), UITaskInit(), PIDInit()
  - SuperCapInit(), INS_Init(), CANCommInit()
  - SubRegister(), PubRegister()

- [x] Task 4: 编写分析文档
  - 输出到 .Doc/ChassisInit分析文档.md
  - 包含流程图和架构图

- [x] Task 5: 整理使用注意事项
  - 双板/单板模式区别
  - 未完成功能标记
  - 宏定义依赖
