#pragma once

#include "main.hpp"

static TaskHandle_t can_task_handle = NULL;

typedef struct can_message_s {
    twai_message_t msg;
    uint64_t timestamp;
} can_message_t;

bool can_setup(void);
void can_check_status(void);
