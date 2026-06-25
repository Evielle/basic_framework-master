/**
 * @file imageRoad.h
 * @author Jiaxing He (1756735553@qq.com)
 * @version 0.1
 * @date 2024-10-20
 *
 */
#ifndef imageRoad_H
#define imageRoad_H

#include "remote_control.h"
#include "stdint.h"
/* ID: 0x0304  Byte:  12    图传链路 */
#pragma pack(1)
typedef  struct
{
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t keyboard_value;
    uint16_t crc16;
}remote_control_t;
#pragma pack()
typedef struct
{
    struct
    {
        int16_t rocker_l_; // 左水平
        int16_t rocker_l1; // 左竖直
        int16_t rocker_r_; // 右水平
        int16_t rocker_r1; // 右竖直
        int16_t wheel;     // 滚轮
        uint8_t mode_sw;  // 水平开关
        uint8_t pause;  // 暂停开关
        uint8_t fn_l;  // 左自定义开关
        uint8_t fn_r;  // 右自定义开关
        uint8_t trigger;  // 扳机
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

    uint8_t key_count[3][16];
} ImageRoad_RC_t;



ImageRoad_RC_t *ImageRoadTaskInit(UART_HandleTypeDef *imageRoad_usart_handle);
void Get_Imageroadcontrol(ImageRoad_RC_t* RC_ctrl);
uint8_t ImageRoadIsOnline(void);

#endif
