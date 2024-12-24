#include "motor.h"
void Motor_Vel_Ctrl(uint8_t addr, int16_t vel, uint8_t acc, bool snF)
{
  if (vel < 0)
    Emm_V5_Vel_Control(addr, 1, (uint16_t)(-vel), acc, snF);
  else
    Emm_V5_Vel_Control(addr, 0, (uint16_t)vel, acc, snF);
}

void Motor_Pos_Ctrl(uint8_t addr, int32_t clk, uint8_t acc, bool snF)
{
  if (clk < 0)
    Emm_V5_Pos_Control(addr, 1, 100, acc, (uint16_t)(-clk), 0, snF);
  else
    Emm_V5_Pos_Control(addr, 0, 100, acc, (uint16_t)clk, 0, snF);
}

void Motor_Turn_Ctrl(int16_t clk, uint8_t acc)
{
  Motor_Pos_Ctrl(1, -clk, acc, 1);
  Motor_Pos_Ctrl(2, -clk, acc, 1);
  Motor_Pos_Ctrl(3, clk, acc, 1);
  Motor_Pos_Ctrl(4, clk, acc, 1);
  Emm_V5_Synchronous_motion(0);
}