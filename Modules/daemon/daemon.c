#include "daemon.h"

#include <string.h>

// daemon 实例静态池; 当前工程各模块统一在这里注册在线监测对象
static DaemonInstance daemon_pool[DAEMON_MAX_NUM];
// 已注册 daemon 实例数量
static uint8_t daemon_count = 0u;

DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config)
{
    DaemonInstance *instance;

    if ((config == NULL) || (daemon_count >= DAEMON_MAX_NUM))
    {
        return NULL;
    }

    // 从静态池中分配一个新的 daemon 实例
    instance = &daemon_pool[daemon_count++];
    memset(instance, 0, sizeof(*instance));

    // reload_count 表示每次喂狗后恢复到的在线计数; 为 0 时至少改为 1
    instance->reload_count = (config->reload_count == 0u) ? 1u : config->reload_count;
    // temp_count 是运行期递减的剩余在线计数
    instance->temp_count = config->init_count;
    // callback 在实例从在线掉到离线时触发
    instance->callback = config->callback;
    // owner_id 仅作为回调或调试时的归属标识
    instance->owner_id = config->owner_id;
    return instance;
}

void DaemonReload(DaemonInstance *instance)
{
    if (instance == NULL)
    {
        return;
    }

    // 模块正常工作一次就把剩余在线计数恢复到满值
    instance->temp_count = instance->reload_count;
}

uint8_t DaemonIsOnline(const DaemonInstance *instance)
{
    // temp_count > 0 表示最近仍在按期喂狗,当前视为在线
    return (uint8_t)((instance != NULL) && (instance->temp_count > 0u));
}

void DaemonTask(void)
{
    // 周期性遍历所有 daemon,统一递减在线计数
    for (uint8_t i = 0; i < daemon_count; ++i)
    {
        DaemonInstance *instance = &daemon_pool[i];

        if (instance->temp_count > 0u)
        {
            instance->temp_count--;
            if (instance->temp_count == 0u)
            {
                // 只在从在线转为离线的这一拍触发一次回调
                if (instance->callback != NULL)
                {
                    instance->callback(instance->owner_id);
                }
            }
        }
    }
}
