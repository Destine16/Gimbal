#include "message_center.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

static Publisher_t topic_pool[MAX_TOPIC_COUNT];           // topic 静态池
static Subscriber_t subscriber_pool[MAX_SUBSCRIBER_COUNT]; // subscriber 静态池
static uint8_t topic_count;                              // 当前已注册 topic 数量
static uint8_t subscriber_count;                         // 当前已注册 subscriber 数量

static void CheckName(char *name)
{
    // topic 名长度超限时直接停机,避免后续字符串越界
    if (strnlen(name, MAX_TOPIC_NAME_LEN + 1) > MAX_TOPIC_NAME_LEN)
    {
        while (1)
        {
        }
    }
}

static void CheckLen(uint8_t len1, uint8_t len2)
{
    // 同名 topic 必须使用同一消息长度
    if (len1 != len2)
    {
        while (1)
        {
        }
    }
}

static void CheckDataLen(uint8_t data_len)
{
    // mailbox 只接受 1~MAX_MESSAGE_DATA_LEN 字节的消息
    if ((data_len == 0u) || (data_len > MAX_MESSAGE_DATA_LEN))
    {
        while (1)
        {
        }
    }
}

static Publisher_t *FindTopic(char *name)
{
    // 在静态 topic 池中按名字查找已注册 topic
    for (uint8_t i = 0; i < topic_count; ++i)
    {
        if (strcmp(topic_pool[i].topic_name, name) == 0)
        {
            return &topic_pool[i];
        }
    }
    return NULL;
}

Publisher_t *PubRegister(char *name, uint8_t data_len)
{
    Publisher_t *pub;

    CheckName(name);
    CheckDataLen(data_len);

    taskENTER_CRITICAL();
    pub = FindTopic(name);
    if (pub != NULL)
    {
        // 同名 topic 已存在时复用它,只补上 publisher 已注册标记
        CheckLen(data_len, pub->data_len);
        pub->pub_registered_flag = 1u;
        taskEXIT_CRITICAL();
        return pub;
    }

    if (topic_count >= MAX_TOPIC_COUNT)
    {
        taskEXIT_CRITICAL();
        while (1)
        {
        }
    }

    pub = &topic_pool[topic_count++];
    memset(pub, 0, sizeof(Publisher_t));
    pub->data_len = data_len;
    strcpy(pub->topic_name, name);
    pub->pub_registered_flag = 1u;
    taskEXIT_CRITICAL();
    return pub;
}

Subscriber_t *SubRegister(char *name, uint8_t data_len)
{
    Subscriber_t *ret; // 本次新分配出来的 subscriber
    Subscriber_t *sub; // 遍历当前 topic 订阅链表时使用的临时指针
    // 先确保同名 topic 已存在,若不存在则一并注册 publisher 侧元数据
    Publisher_t *pub = PubRegister(name, data_len);

    taskENTER_CRITICAL();
    if (subscriber_count >= MAX_SUBSCRIBER_COUNT)
    {
        taskEXIT_CRITICAL();
        while (1)
        {
        }
    }

    // 从静态 subscriber 池中取出一个空槽作为新的订阅者 mailbox
    ret = &subscriber_pool[subscriber_count++];
    memset(ret, 0, sizeof(Subscriber_t));
    ret->data_len = data_len;

    if (pub->first_subs == NULL)
    {
        // 当前是该 topic 的第一个订阅者,直接挂到链表头
        pub->first_subs = ret;
        taskEXIT_CRITICAL();
        return ret;
    }

    // 否则沿链表走到末尾,把新订阅者挂进去
    sub = pub->first_subs;
    while (sub->next_subs_queue != NULL)
    {
        sub = sub->next_subs_queue;
    }
    sub->next_subs_queue = ret;
    taskEXIT_CRITICAL();
    return ret;
}

uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr)
{
    uint8_t has_message;

    if ((sub == NULL) || (data_ptr == NULL))
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    has_message = sub->valid;
    if (has_message)
    {
        // mailbox 模式: 每个 subscriber 只保留一份最新消息,取走后清空 valid
        memcpy(data_ptr, sub->mailbox, sub->data_len);
        sub->valid = 0u;
    }
    taskEXIT_CRITICAL();
    return has_message;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr)
{
    Subscriber_t *iter;

    if ((pub == NULL) || (data_ptr == NULL))
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    iter = pub->first_subs;
    while (iter != NULL)
    {
        // mailbox 模式下直接覆盖成最新消息,旧消息不会排队保留
        memcpy(iter->mailbox, data_ptr, pub->data_len);
        iter->valid = 1u;
        iter = iter->next_subs_queue;
    }
    taskEXIT_CRITICAL();

    return 1u;
}
