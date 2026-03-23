#ifndef MESSAGE_CENTER_H
#define MESSAGE_CENTER_H

#include <stdint.h>

#define MAX_TOPIC_NAME_LEN 32
#define MAX_TOPIC_COUNT 8
#define MAX_SUBSCRIBER_COUNT 8
#define MAX_MESSAGE_DATA_LEN 128

typedef struct mqt
{
    // 每个订阅者只保留最新一条消息
    uint8_t mailbox[MAX_MESSAGE_DATA_LEN];
    uint8_t data_len;
    uint8_t valid;
    struct mqt *next_subs_queue;
} Subscriber_t;

typedef struct ent
{
    char topic_name[MAX_TOPIC_NAME_LEN + 1];
    uint8_t data_len;
    Subscriber_t *first_subs;
    struct ent *next_topic_node;
    uint8_t pub_registered_flag;
} Publisher_t;

Subscriber_t *SubRegister(char *name, uint8_t data_len);
Publisher_t *PubRegister(char *name, uint8_t data_len);
uint8_t SubGetMessage(Subscriber_t *sub, void *data_ptr);
uint8_t PubPushMessage(Publisher_t *pub, void *data_ptr);

#endif
