#include "daemon.h"

#include <string.h>

static DaemonInstance daemon_pool[DAEMON_MAX_NUM];
static uint8_t daemon_count = 0u;

DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config)
{
    DaemonInstance *instance;

    if ((config == NULL) || (daemon_count >= DAEMON_MAX_NUM))
    {
        return NULL;
    }

    instance = &daemon_pool[daemon_count++];
    memset(instance, 0, sizeof(*instance));

    instance->reload_count = (config->reload_count == 0u) ? 1u : config->reload_count;
    instance->temp_count = config->init_count;
    instance->callback = config->callback;
    instance->owner_id = config->owner_id;
    return instance;
}

void DaemonReload(DaemonInstance *instance)
{
    if (instance == NULL)
    {
        return;
    }

    instance->temp_count = instance->reload_count;
}

uint8_t DaemonIsOnline(const DaemonInstance *instance)
{
    return (uint8_t)((instance != NULL) && (instance->temp_count > 0u));
}

void DaemonTask(void)
{
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
