#include "transmit.hpp"

bool esp_now_setup() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    memcpy(_peer_info.peer_addr, _broadcast_addr, 6);
    _peer_info.channel = 0;
    _peer_info.encrypt = false;
    if (esp_now_add_peer(&_peer_info) != ESP_OK) {
        return false;
    }
    return true;
}

esp_err_t esp_now_send_packet(esp_now_packet_t* pkt) {
    esp_err_t err;
    const int max_retries = 3;
    int retry_count = 0;

    do {
        err = esp_now_send(_broadcast_addr, (uint8_t*)pkt, sizeof(esp_now_packet_t));

        if (err == ESP_ERR_ESPNOW_NO_MEM) {
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(2 * retry_count));  // Increasing backoff
            debugf("ESP-NOW memory error, retry %d of %d\n", retry_count, max_retries);
        }
    } while (err == ESP_ERR_ESPNOW_NO_MEM && retry_count < max_retries);

    if (err != ESP_OK) {
        debugf("Failed to send CAN msg after %d attempts: %d\n", retry_count, err);
    } else {
        debugln("ESPNOW SEND SUCCESS");
    }

    return err;
}

esp_err_t esp_now_send_batch(const esp_now_batch_t* batch) {
    esp_err_t result = esp_now_send(
        _broadcast_addr,
        (uint8_t*)batch,
        batch->count * sizeof(esp_now_packet_t) + sizeof(uint8_t)
    );
    return result;
}
