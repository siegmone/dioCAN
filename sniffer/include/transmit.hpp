#pragma once

#include "main.hpp"

static uint8_t _broadcast_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static esp_now_peer_info_t _peer_info;
static int _esp_now_mesh_id;

static TaskHandle_t esp_now_task_handle = NULL;

typedef struct esp_now_packet_s {
    uint64_t timestamp;
    uint8_t data[TWAI_FRAME_MAX_DLC];
    uint32_t can_id;
    int mesh_id;
    uint8_t dlc;
} esp_now_packet_t;

#define MAX_BATCH_SIZE 5

typedef struct {
    esp_now_packet_t msgs[MAX_BATCH_SIZE];
    uint8_t count;
} esp_now_batch_t;

bool esp_now_setup(void);
esp_err_t esp_now_send_packet(esp_now_packet_t* pkt);
esp_err_t esp_now_send_batch(const esp_now_batch_t* batch);
