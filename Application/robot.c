#include "robot.h"

#include "bsp_dwt.h"
#include "gimbal.h"
#include "robot_cmd.h"

void RobotInit(void)
{
    DWT_Init(HAL_RCC_GetHCLKFreq() / 1000000U);
    GimbalInit();
    RobotCMDInit();
}

void RobotTask(void)
{
    RobotCMDTask();
    GimbalTask();
}
