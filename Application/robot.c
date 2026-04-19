#include "robot.h"

#include "gimbal.h"
#include "robot_cmd.h"

void RobotInit(void)
{
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
