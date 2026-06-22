#ifndef CHASSIS_H
#define CHASSIS_H

// typedef struct{
//     float target_chassis_vx;
//     float target_chassis_vy;
//     float target_chassis_wz;
//     float real_chassis_vx;
//     float real_chassis_vy;
//     float real_chassis_wz;

// }Chassis_Velocity_s;
/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

#endif // CHASSIS_H