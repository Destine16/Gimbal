#ifndef GM6020_INTERNAL_H
#define GM6020_INTERNAL_H

#include "gm6020.h"

extern GM6020_Instance gm6020_list[GM6020_MAX_NUM];
extern uint8_t gm6020_count;

// 将离线超时毫秒数换算成 daemon reload 计数
uint16_t GM6020_DaemonReloadCount(void);
// 优先用 daemon 判断在线; 未注册 daemon 时退回 tick 超时判断
uint8_t GM6020_RuntimeOnline(const GM6020_Instance *motor, uint32_t now_tick);
// 根据电机 id 决定发往 0x1FF 还是 0x2FF,以及对应 8 字节中的位置
void GM6020_AssignTxGroup(GM6020_Instance *motor);

// 清空三环 PID 的积分/微分状态
void GM6020_ResetControlState(GM6020_Instance *motor);
// 逐个电机更新本轮输出命令
void GM6020_UpdateAll(void);
// 按 CAN 总线分组打包并发送全部输出命令
void GM6020_SendAll(void);
// 将一帧 CAN 反馈解析回 GM6020 测量量
void GM6020_ParseFeedback(GM6020_Instance *motor, const uint8_t rx_data[8], uint32_t now_tick);

#endif
