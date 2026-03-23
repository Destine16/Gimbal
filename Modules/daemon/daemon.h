#ifndef DAEMON_H
#define DAEMON_H

#include <stdint.h>

#define DAEMON_MAX_NUM 16u

typedef void (*offline_callback)(void *);

typedef struct
{
    uint16_t reload_count;     // 每次喂狗后恢复到的计数值
    uint16_t temp_count;       // 当前剩余计数,减到 0 说明超时
    offline_callback callback; // 从在线转为离线时调用一次
    void *owner_id;            // 交给回调识别具体实例
} DaemonInstance;

typedef struct
{
    uint16_t reload_count;     // 正常工作时的超时计数
    uint16_t init_count;       // 启动阶段的初始计数; 0 表示默认离线
    offline_callback callback; // 离线回调,可为空
    void *owner_id;            // 拥有该 daemon 的对象地址
} Daemon_Init_Config_s;

DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config);
void DaemonReload(DaemonInstance *instance);
uint8_t DaemonIsOnline(const DaemonInstance *instance);
void DaemonTask(void);

#endif
