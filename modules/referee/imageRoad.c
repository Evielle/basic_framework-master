/**
 * @file imageRoad.c
 * @author Jiaxing He (1756735553@qq.com)
 * @version 0.1
 * @date 2024-10-20
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"
#include "crc_ref.h"
#include "bsp_usart.h"
#include "task.h"
#include "daemon.h"
#include "bsp_log.h"
#include "imageRoad.h"
#include "referee_protocol.h"

#define RE_RX_BUFFER_SIZE 21u // 裁判系统接收缓冲区大小

static USARTInstance *imageRoad_usart_instance; // 裁判系统串口实例
static remote_control_t *ImageRoad_info,imageRoad_info;
static DaemonInstance *imageRoad_daemon;
static ImageRoad_RC_t imageRoad_rc[2];

/**
 * @brief 图传数据解析
 *
 */
static void KeyParse()
{
    // for(int i = 0; i < 16; i++)
    // {  
    //     感觉这样延迟更高
    //     if(imageRoad_data->keyboard_value&(1<<i)){
    //         imageRoad_key[TEMP].key[KEY_PRESS][i]=1;
    //     }
    //     else{
    //         imageRoad_key[TEMP].key[KEY_PRESS][i]=0;
    //     }
    // }
    //改成位域
    imageRoad_rc[TEMP].mouse.x=(int16_t)(imageRoad_info.mouse_x);
    imageRoad_rc[TEMP].mouse.y=(int16_t)(imageRoad_info.mouse_y);
    imageRoad_rc[TEMP].mouse.press_l=(uint8_t)(imageRoad_info.mouse_left);
    imageRoad_rc[TEMP].mouse.press_r=(uint8_t)(imageRoad_info.mouse_right);
    
    *(uint16_t *)&imageRoad_rc[TEMP].key[KEY_PRESS] = (uint16_t)(imageRoad_info.keyboard_value);  //改为此方法，更迅速，为减少延迟
    if (imageRoad_rc[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        imageRoad_rc[TEMP].key[KEY_PRESS_WITH_CTRL] = imageRoad_rc[TEMP].key[KEY_PRESS];
    else
        memset(&imageRoad_rc[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (imageRoad_rc[TEMP].key[KEY_PRESS].shift) // shift键按下
        imageRoad_rc[TEMP].key[KEY_PRESS_WITH_SHIFT] = imageRoad_rc[TEMP].key[KEY_PRESS];
    else
        memset(&imageRoad_rc[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));
    //计数解析
    uint16_t key_now = imageRoad_rc[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = imageRoad_rc[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = imageRoad_rc[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = imageRoad_rc[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = imageRoad_rc[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = imageRoad_rc[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下
    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            imageRoad_rc[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            imageRoad_rc[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            imageRoad_rc[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&imageRoad_rc[LAST], &imageRoad_rc[TEMP], sizeof(ImageRoad_RC_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

static void ImageRoadLostCallback(void *arg)
{
	USARTServiceInit(imageRoad_usart_instance);
	LOGWARNING("[rm_img] lost referee data");
}

static void JudgeReadData_imageRoad(uint8_t *buff)
{
	if (buff == NULL)
		return;
	if (buff[0]==0xA9&&buff[1]==0x53)
	{
        if(Verify_CRC16_Check_Sum(buff,21))
        {
			memcpy(&imageRoad_info, (buff+2), 19);
        }
	}
}

static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&imageRoad_rc[TEMP].rc.rocker_l_ + i)) > 660)
            *(&imageRoad_rc[TEMP].rc.rocker_l_ + i) = 0;
}

static void remoteParse()
{
    imageRoad_rc[TEMP].rc.rocker_r_ = (int16_t)(imageRoad_info.ch_0) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    imageRoad_rc[TEMP].rc.rocker_r1 = (int16_t)(imageRoad_info.ch_1) - RC_CH_VALUE_OFFSET;                              //!< Channel 1
    imageRoad_rc[TEMP].rc.rocker_l1 = (int16_t)(imageRoad_info.ch_2) - RC_CH_VALUE_OFFSET;                              //!< Channel 2
    imageRoad_rc[TEMP].rc.rocker_l_ = (int16_t)(imageRoad_info.ch_3) - RC_CH_VALUE_OFFSET;                              //!< Channel 3
    imageRoad_rc[TEMP].rc.wheel = (int16_t)(imageRoad_info.wheel) - RC_CH_VALUE_OFFSET;
    RectifyRCjoystick();
    imageRoad_rc[TEMP].rc.mode_sw =(uint8_t)(imageRoad_info.mode_sw);
    imageRoad_rc[TEMP].rc.pause =(uint8_t)(imageRoad_info.pause);
    imageRoad_rc[TEMP].rc.fn_l =(uint8_t)(imageRoad_info.fn_1);
    imageRoad_rc[TEMP].rc.fn_r =(uint8_t)(imageRoad_info.fn_2);
    imageRoad_rc[TEMP].rc.trigger =(uint8_t)(imageRoad_info.trigger);
    imageRoad_rc[TEMP].mouse.press_l=(uint8_t)(imageRoad_info.mouse_left);
    imageRoad_rc[TEMP].mouse.press_r=(uint8_t)(imageRoad_info.mouse_right);
    imageRoad_rc[TEMP].mouse.x=(int16_t)(imageRoad_info.mouse_x);
    imageRoad_rc[TEMP].mouse.y=(int16_t)(imageRoad_info.mouse_y);
    memcpy(&imageRoad_rc[LAST], &imageRoad_rc[TEMP], sizeof(ImageRoad_RC_t));
}

static void ImageRoadRxCallback()
{
	DaemonReload(imageRoad_daemon);
	JudgeReadData_imageRoad(imageRoad_usart_instance->recv_buff);
    remoteParse();
    KeyParse();
}
ImageRoad_RC_t *ImageRoadTaskInit(UART_HandleTypeDef *imageRoad_usart_handle)
{
	USART_Init_Config_s conf;
	conf.module_callback = ImageRoadRxCallback;
	conf.usart_handle = imageRoad_usart_handle;
	conf.recv_buff_size = 21u; // mx 255(u8)
	imageRoad_usart_instance = USARTRegister(&conf);

	Daemon_Init_Config_s daemon_conf = {
		.callback = ImageRoadLostCallback,
		.owner_id = imageRoad_usart_instance,
		.reload_count = 30, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	imageRoad_daemon = DaemonRegister(&daemon_conf);

	return imageRoad_rc;
}

// ImageRoad_RC_t *ImageRoadTaskInit(UART_HandleTypeDef *imageRoad_usart_handle)
// {
//     ima = ImageRoadInit(imageRoad_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
//     return ImageRoad_info;
// }

void Get_Imageroadcontrol(ImageRoad_RC_t* RC_ctrl)
{
    memcpy(RC_ctrl,imageRoad_rc,2*sizeof(ImageRoad_RC_t));
}

uint8_t ImageRoadIsOnline(void)
{
    return DaemonIsOnline(imageRoad_daemon);
}
