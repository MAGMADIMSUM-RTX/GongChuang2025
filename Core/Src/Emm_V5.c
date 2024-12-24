#include "can.h"
#include "Emm_V5.h"
#include "stdbool.h"
#include "usart.h"

#define MOTOR_CAN hcan2

CAN_t can = {0};
int32_t Motor1_Pos = 0;
CAN_TxHeaderTypeDef chassis_tx_message;

// /**
//  * @brief   CAN1_RX0接收中断
//  * @param   无
//  * @retval  无
//  */
// void USB_LP_CAN1_RX0_IRQHandler(void)
// {
//     // 接收一包数据
//     CAN_Receive(CAN1, CAN_RX_FIFO0, (CAN_RxHeaderTypeDef *)(&rx_header));

//     // 一帧数据接收完成，置位帧标志位
//     can.rxFrameFlag = true;
// }
/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[16];
    uint8_t txmessage[20];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if (rx_data[0] == 0x36) // 读取位置
    {
        Motor1_Pos = (int32_t)((int32_t)rx_data[2] << 24 | (int32_t)rx_data[3] << 16 | (int32_t)rx_data[4] << 8 | (int32_t)rx_data[5]);
        if (rx_data[1] == 0x01)
        {
            Motor1_Pos = -Motor1_Pos;
        }
    }

    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
}

/**
 * @brief   CAN发送多个字节
 * @param   无
 * @retval  无
 */
void can_SendCmd(uint8_t *cmd, uint8_t len)
{
    uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;
    uint8_t Data[len];

    // 除去ID地址和功能码后的数据长度
    j = len - 2;

    // 发送数据
    while (i < j)
    {
        uint32_t send_mail_box;
        // 数据个数
        k = j - i;

        // 填充缓存
        chassis_tx_message.StdId = 0x00;
        chassis_tx_message.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
        Data[0] = cmd[1];
        chassis_tx_message.IDE = CAN_ID_EXT;
        chassis_tx_message.RTR = CAN_RTR_DATA;

        // 小于8字节命令
        if (k < 8)
        {
            for (l = 0; l < k; l++, i++)
            {
                Data[l + 1] = cmd[i + 2];
            }
            chassis_tx_message.DLC = k + 1;
        }
        // 大于8字节命令，分包发送，每包数据最多发送8个字节
        else
        {
            for (l = 0; l < 7; l++, i++)
            {
                Data[l + 1] = cmd[i + 2];
            }
            chassis_tx_message.DLC = 8;
        }

        // 发送数据
        // CAN_Transmit(CAN1, (CAN_TxHeaderTypeDef *)(&chassis_tx_message));
        HAL_CAN_AddTxMessage(&MOTOR_CAN, &chassis_tx_message, Data, &send_mail_box);

        // 记录发送的第几包的数据
        ++packNum;
    }
}

/**********************************************************/

/*
 * @brief    将当前位置清零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0x0A; // 功能码
    cmd[2] = 0x6D; // 辅助码
    cmd[3] = 0x6B; // 校验字节

    // 发送命令
    can_SendCmd(cmd, 4);
}

/**
 * @brief    解除堵转保护
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0x0E; // 功能码
    cmd[2] = 0x52; // 辅助码
    cmd[3] = 0x6B; // 校验字节

    // 发送命令
    can_SendCmd(cmd, 4);
}

/**
 * @brief    读取系统参数
 * @param    addr  ：电机地址
 * @param    s     ：系统参数类型
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[i] = addr;
    ++i; // 地址

    switch (s) // 功能码
    {
    case S_VER:
        cmd[i] = 0x1F;
        ++i;
        break;
    case S_RL:
        cmd[i] = 0x20;
        ++i;
        break;
    case S_PID:
        cmd[i] = 0x21;
        ++i;
        break;
    case S_VBUS:
        cmd[i] = 0x24;
        ++i;
        break;
    case S_CPHA:
        cmd[i] = 0x27;
        ++i;
        break;
    case S_ENCL:
        cmd[i] = 0x31;
        ++i;
        break;
    case S_TPOS:
        cmd[i] = 0x33;
        ++i;
        break;
    case S_VEL:
        cmd[i] = 0x35;
        ++i;
        break;
    case S_CPOS:
        cmd[i] = 0x36;
        ++i;
        break;
    case S_PERR:
        cmd[i] = 0x37;
        ++i;
        break;
    case S_FLAG:
        cmd[i] = 0x3A;
        ++i;
        break;
    case S_ORG:
        cmd[i] = 0x3B;
        ++i;
        break;
    case S_Conf:
        cmd[i] = 0x42;
        ++i;
        cmd[i] = 0x6C;
        ++i;
        break;
    case S_State:
        cmd[i] = 0x43;
        ++i;
        cmd[i] = 0x7A;
        ++i;
        break;
    default:
        break;
    }

    cmd[i] = 0x6B;
    ++i; // 校验字节

    // 发送命令
    can_SendCmd(cmd, i);
}

/**
 * @brief    修改开环/闭环控制模式
 * @param    addr     ：电机地址
 * @param    svF      ：是否存储标志，false为不存储，true为存储
 * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;      // 地址
    cmd[1] = 0x46;      // 功能码
    cmd[2] = 0x69;      // 辅助码
    cmd[3] = svF;       // 是否存储标志，false为不存储，true为存储
    cmd[4] = ctrl_mode; // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
    cmd[5] = 0x6B;      // 校验字节

    // 发送命令
    can_SendCmd(cmd, 6);
}

/**
 * @brief    使能信号控制
 * @param    addr  ：电机地址
 * @param    state ：使能状态     ，true为使能电机，false为关闭电机
 * @param    snF   ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;           // 地址
    cmd[1] = 0xF3;           // 功能码
    cmd[2] = 0xAB;           // 辅助码
    cmd[3] = (uint8_t)state; // 使能状态
    cmd[4] = snF;            // 多机同步运动标志
    cmd[5] = 0x6B;           // 校验字节

    // 发送命令
    can_SendCmd(cmd, 6);
}

/**
 * @brief    速度模式
 * @param    addr：电机地址
 * @param    dir ：方向       ，0为CW，其余值为CCW
 * @param    vel ：速度       ，范围0 - 5000RPM
 * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;                // 地址
    cmd[1] = 0xF6;                // 功能码
    cmd[2] = dir;                 // 方向
    cmd[3] = (uint8_t)(vel >> 8); // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0); // 速度(RPM)低8位字节
    cmd[5] = acc;                 // 加速度，注意：0是直接启动
    cmd[6] = snF;                 // 多机同步运动标志
    cmd[7] = 0x6B;                // 校验字节

    // 发送命令
    can_SendCmd(cmd, 8);
}

/**
 * @brief    位置模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
 * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;                 // 地址
    cmd[1] = 0xFD;                 // 功能码
    cmd[2] = dir;                  // 方向
    cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
    cmd[5] = acc;                  // 加速度，注意：0是直接启动
    cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
    cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
    cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
    cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )
    cmd[10] = raF;                 // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                 // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                // 校验字节

    // 发送命令
    can_SendCmd(cmd, 13);
}

/**
 * @brief    立即停止（所有控制模式都通用）
 * @param    addr  ：电机地址
 * @param    snF   ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0xFE; // 功能码
    cmd[2] = 0x98; // 辅助码
    cmd[3] = snF;  // 多机同步运动标志
    cmd[4] = 0x6B; // 校验字节

    // 发送命令
    can_SendCmd(cmd, 5);
}

/**
 * @brief    多机同步运动
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0xFF; // 功能码
    cmd[2] = 0x66; // 辅助码
    cmd[3] = 0x6B; // 校验字节

    // 发送命令
    can_SendCmd(cmd, 4);
}

/**
 * @brief    设置单圈回零的零点位置
 * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0x93; // 功能码
    cmd[2] = 0x88; // 辅助码
    cmd[3] = svF;  // 是否存储标志，false为不存储，true为存储
    cmd[4] = 0x6B; // 校验字节

    // 发送命令
    can_SendCmd(cmd, 5);
}

/**
 * @brief    修改回零参数
 * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
 * @param    o_dir  ：回零方向，0为CW，其余值为CCW
 * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
 * @param    o_tm   ：回零超时时间，单位：毫秒
 * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
 * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
 * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
 * @param    potF   ：上电自动触发回零，false为不使能，true为使能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    uint8_t cmd[32] = {0};

    // 装载命令
    cmd[0] = addr;                    // 地址
    cmd[1] = 0x4C;                    // 功能码
    cmd[2] = 0xAE;                    // 辅助码
    cmd[3] = svF;                     // 是否存储标志，false为不存储，true为存储
    cmd[4] = o_mode;                  // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[5] = o_dir;                   // 回零方向
    cmd[6] = (uint8_t)(o_vel >> 8);   // 回零速度(RPM)高8位字节
    cmd[7] = (uint8_t)(o_vel >> 0);   // 回零速度(RPM)低8位字节
    cmd[8] = (uint8_t)(o_tm >> 24);   // 回零超时时间(bit24 - bit31)
    cmd[9] = (uint8_t)(o_tm >> 16);   // 回零超时时间(bit16 - bit23)
    cmd[10] = (uint8_t)(o_tm >> 8);   // 回零超时时间(bit8  - bit15)
    cmd[11] = (uint8_t)(o_tm >> 0);   // 回零超时时间(bit0  - bit7 )
    cmd[12] = (uint8_t)(sl_vel >> 8); // 无限位碰撞回零检测转速(RPM)高8位字节
    cmd[13] = (uint8_t)(sl_vel >> 0); // 无限位碰撞回零检测转速(RPM)低8位字节
    cmd[14] = (uint8_t)(sl_ma >> 8);  // 无限位碰撞回零检测电流(Ma)高8位字节
    cmd[15] = (uint8_t)(sl_ma >> 0);  // 无限位碰撞回零检测电流(Ma)低8位字节
    cmd[16] = (uint8_t)(sl_ms >> 8);  // 无限位碰撞回零检测时间(Ms)高8位字节
    cmd[17] = (uint8_t)(sl_ms >> 0);  // 无限位碰撞回零检测时间(Ms)低8位字节
    cmd[18] = potF;                   // 上电自动触发回零，false为不使能，true为使能
    cmd[19] = 0x6B;                   // 校验字节

    // 发送命令
    can_SendCmd(cmd, 20);
}

/**
 * @brief    触发回零
 * @param    addr   ：电机地址
 * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
 * @param    snF   ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;   // 地址
    cmd[1] = 0x9A;   // 功能码
    cmd[2] = o_mode; // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[3] = snF;    // 多机同步运动标志，false为不启用，true为启用
    cmd[4] = 0x6B;   // 校验字节

    // 发送命令
    can_SendCmd(cmd, 5);
}

/**
 * @brief    强制中断并退出回零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0x9C; // 功能码
    cmd[2] = 0x48; // 辅助码
    cmd[3] = 0x6B; // 校验字节

    // 发送命令
    can_SendCmd(cmd, 4);
}
