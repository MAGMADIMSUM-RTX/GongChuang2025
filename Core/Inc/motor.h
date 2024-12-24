/*
 * @Author: MAGMADIMSUM madmaliu@bupt.edu.cn
 * @Date: 2024-12-12 22:50:13
 * @LastEditors: MAGMADIMSUM madmaliu@bupt.edu.cn
 * @LastEditTime: 2024-12-12 22:52:51
 * @FilePath: \project\Core\Inc\motor.h
 * @Description: 
 * 
 */
/*
 * @Author: MAGMADIMSUM madmaliu@bupt.edu.cn
 * @Date: 2024-12-12 22:50:13
 * @LastEditors: MAGMADIMSUM madmaliu@bupt.edu.cn
 * @LastEditTime: 2024-12-12 22:50:26
 * @FilePath: \project\Core\Inc\motor.h
 * @Description:
 *
 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "stdbool.h"
#include "string.h"

void Motor_Turn_Ctrl(int16_t clk, uint8_t acc);
void Motor_Pos_Ctrl(uint8_t addr, int32_t clk, uint8_t acc, bool snF);
void Motor_Vel_Ctrl(uint8_t addr, int16_t vel, uint8_t acc, bool snF);

#endif // __MOTOR_H__