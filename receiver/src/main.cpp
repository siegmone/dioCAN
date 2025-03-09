#define DEBUG

#ifdef DEBUG
#define debug(x)       Serial.print(x)
#define debugln(x)     Serial.println(x)
#define debugf(fmt, x) Serial.printf(fmt, x)
#else
#define debug(x)
#define debugln(x)
#define debugf(fmt, x)
#endif

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define MESH_ID 18460

typedef struct esp_now_frame_s {
    int mesh_id;
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} esp_now_frame_t;

esp_now_frame_t esp_now_frame;

#define SERIAL_MAX_BUFF_LEN 30
char slcan_com[SERIAL_MAX_BUFF_LEN];
bool send_ok = true;
bool got_new_can_msg = false;
bool send_can_msgs = true;
bool send_timestamp = true;

void handleSerial();
void send_can_over_serial();
void send_ack();
void send_nack();
void recv_cb(const uint8_t *mac, const uint8_t *data, int len);

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    delay(10);
    while (esp_now_init() != ESP_OK) {
        debugln("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(recv_cb);
}

void loop() {
    // debugln(WiFi.macAddress());
    send_can_over_serial();
    handleSerial();
    delay(100);
}

void recv_cb(const uint8_t *mac, const uint8_t *data, int len) {
    if (got_new_can_msg)
        return;
    memcpy(&esp_now_frame, data, sizeof(esp_now_frame));
    got_new_can_msg = true;  // TODO: queue the msgs
}

void send_can_over_serial() {
    char buf[45];  // Increased size for safety
    int idx = 0;

    /*
        if (esp_frame.can_id <= 0x7FF) {
          // Standard 11-bit CAN ID
          idx += snprintf(buf + idx, sizeof(buf) - idx, "t%03x",
       esp_frame.can_id); } else if (esp_frame.can_id <= 0x1FFFFFFF) {
          // Extended 29-bit CAN ID
          idx += snprintf(buf + idx, sizeof(buf) - idx, "T%08x",
       esp_frame.can_id); } else {
          // Invalid CAN ID
          return;
        }
    */

    idx +=
        snprintf(buf + idx, sizeof(buf) - idx, "t%03x", esp_now_frame.can_id);
    idx += snprintf(buf + idx, sizeof(buf) - idx, "%d", esp_now_frame.dlc);
    for (int i = 0; i < esp_now_frame.dlc; i++) {
        idx += snprintf(buf + idx, sizeof(buf) - idx, "%02x",
                        esp_now_frame.data[i]);
    }

    buf[idx++] = '\r';  // Carriage return
    buf[idx++] = '\0';  // Null-terminate the buffer
    Serial.print(buf);
}

void handleSerial() {
    if (Serial.available() == 0)
        return;

    Serial.readBytesUntil(13, slcan_com, SERIAL_MAX_BUFF_LEN);

    switch (slcan_com[0]) {
        case 'S':
        case 's':
        case 'R':
        case 'r':
        case 'P':
        case 'A':
        case 'x':
        case 'U':
            // Serial.begin
            break;

        case 't':
            Serial.print("z");
            break;

        case 'T':
            Serial.print("Z");
            break;

        case 'O':
        case 'L':
            send_can_msgs = true;
            break;

        case 'C':
            send_can_msgs = false;
            break;

        case 'F':
            Serial.print("F00");
            break;

        case 'V':
        case 'v':
            Serial.print("V1013\n");
            break;

        case 'N':
            Serial.print("NA123\n");
            break;

        case 'B':
            Serial.print("CAN1");
            Serial.print("\n");
            break;

        case 'Z':
            if (slcan_com[1] == '1')
                send_timestamp = true;
            if (slcan_com[1] == '0')
                send_timestamp = false;
            break;

        case 'X':
            break;

        default:
            send_ok = false;
            break;
    }
    if (send_ok)
        send_ack();
    send_ok = true;
}

void send_ack() {
    Serial.write('\r');  // ACK
}

void send_nack() {
    Serial.write('\a');  // NACK
}
