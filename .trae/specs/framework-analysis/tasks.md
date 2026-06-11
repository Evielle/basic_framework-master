# Tasks

- [x] Task 1: 分析 main() 函数执行流程
  - 梳理 main.c 中从 HAL_Init() 到 osKernelStart() 的完整流程
  - 列出所有外设初始化函数

- [x] Task 2: 分析 RobotInit() 及相关初始化函数
  - 分析 BSPInit() 初始化的组件
  - 分析 RobotCMDInit(), ChassisInit() 等可选初始化
  - 分析 OSTaskInit() 创建的任务

- [x] Task 3: 分析 FreeRTOS 任务结构
  - 列出所有任务及其优先级和频率
  - 分析每个任务的职责

- [x] Task 4: 编写框架分析文档
  - 输出到 .Doc/框架分析_main_c.md
  - 包含架构图和功能说明
