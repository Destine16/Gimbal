#include "robot_task.h"

#include "cmsis_os.h"

#include "daemon.h"
#include "gm6020.h"
#include "ins_task.h"
#include "robot_def.h"

void StartINSTask(void *argument)
{
    (void)argument;

    INS_Init();

    for (;;)
    {
        INS_Task();
        osDelay(1);
    }
}

void StartMotorTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        GM6020_ControlAll();
        osDelay(1);
    }
}

void StartDaemonTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        // 低频统一处理各模块在线计数
        DaemonTask();
        osDelay(DAEMON_TASK_PERIOD_MS);
    }
}
