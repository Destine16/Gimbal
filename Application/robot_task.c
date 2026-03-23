#include "robot_task.h"

#include "cmsis_os.h"

#include "daemon.h"
#include "gm6020.h"
#include "ins_task.h"
#include "robot_def.h"

void StartINSTask(void *argument)
{
    (void)argument;

    // insTask 启动时先执行一次 INS_Init(),后续只保留 1ms 周期更新
    INS_Init();

    for (;;)
    {
        // 姿态任务周期: 1ms
        // 职责: 读取最新 IMU 样本并更新姿态解算结果
        INS_Task();
        osDelay(1);
    }
}

void StartMotorTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        // 电机任务周期: 1ms
        // 职责: 更新所有 GM6020 控制环并统一发出本轮 CAN 控制帧
        GM6020_ControlAll();
        osDelay(1);
    }
}

void StartDaemonTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        // 守护任务周期: DAEMON_TASK_PERIOD_MS
        // 职责: 低频统一处理各模块在线计数和离线回调
        DaemonTask();
        osDelay(DAEMON_TASK_PERIOD_MS);
    }
}
