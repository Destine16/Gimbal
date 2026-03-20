#include "message_center.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

static Publisher_t topic_pool[MAX_TOPIC_COUNT];
static Subscriber_t subscriber_pool[MAX_SUBSCRIBER_COUNT];
static uint8_t topic_count;
static uint8_t subscriber_count;

static void CheckName(char *name)
{
    if (strnlen(name, MAX_TOPIC_NAME_LEN + 1) > MAX_TOPIC_NAME_LEN)
    {
        while (1)
        {
        }
    }
}

static void CheckLen(uint8_t len1, uint8_t len2)
{
    if (len1 != len2)
    {
        while (1)
        {
        }
    }
}

static void CheckDataLen(uint8_t data_len)
{
    if ((data_len == 0u) || (data_len > MAX_MESSAGE_DATA_LEN))
    {
        while (1)
        {
        }
    }
}

static Publisher_t *FindTopic(char *name)
{
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
    Subscriber_t *ret;
    Subscriber_t *sub;
    Publisher_t *pub = PubRegister(name, data_len);

    taskENTER_CRITICAL();
    if (subscriber_count >= MAX_SUBSCRIBER_COUNT)
    {
        taskEXIT_CRITICAL();
        while (1)
        {
        }
    }

    ret = &subscriber_pool[subscriber_count++];
    memset(ret, 0, sizeof(Subscriber_t));
    ret->data_len = data_len;

    if (pub->first_subs == NULL)
    {
        pub->first_subs = ret;
        taskEXIT_CRITICAL();
        return ret;
    }

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
        memcpy(data_ptr, sub->mailbox, sub->data_len);
        sub->valid = 0u;
    }
    taskEXIT_CRITICAL();
    return has_message;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr)
{
    Subscriber_t *iter = pub->first_subs;

    if ((pub == NULL) || (data_ptr == NULL))
    {
        return 0u;
    }

    taskENTER_CRITICAL();
    while (iter != NULL)
    {
        memcpy(iter->mailbox, data_ptr, pub->data_len);
        iter->valid = 1u;
        iter = iter->next_subs_queue;
    }
    taskEXIT_CRITICAL();

    return 1u;
}
