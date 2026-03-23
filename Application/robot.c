#include "robot.h"

#include "bsp_dwt.h"
#include "gimbal.h"
#include "robot_cmd.h"

void RobotInit(void)
{
    DWT_Init(HAL_RCC_GetHCLKFreq() / 1000000U);
    // 应用层主消息流:
    // robot_cmd -> gimbal_cmd -> gimbal
    // gimbal -> gimbal_feed -> robot_cmd
    GimbalInit();
    RobotCMDInit();
}

void RobotTask(void)
{
    RobotCMDTask();
    GimbalTask();
}
