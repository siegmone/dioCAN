/*
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include <algorithm>
#include <vector>

#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/twai.h"

// DEBUG INFO OPTION
// decomment to display debug informations
#define DEBUG

#ifdef DEBUG
#define debug(x)       Serial.print(x)
#define debugln(x)     Serial.println(x)
#define debugf(fmt, x) Serial.printf(fmt, x)
#else
#define debug(x)
#define debugln(x)
#define debugf(fmt, x)
#endif  // DEBUG

// timeouts
#define POLLING_RATE_MS  100
#define ERROR_TIMEOUT_MS 500

// pin definitions
#define GPIO_ERROR_LED GPIO_NUM_22
#define GPIO_CAN_TX    GPIO_NUM_18
#define GPIO_CAN_RX    GPIO_NUM_19

// ESP-NOW communication
typedef struct esp_now_frame_s {
    int mesh_id;
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} esp_now_frame_t;

static esp_now_frame_t esp_now_frame;

static uint8_t broadcast_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static esp_now_peer_info_t peer_info;
static int esp_now_mesh_id;

static void esp_now_send_can_msg(twai_message_t* msg) {
    esp_err_t err;
    esp_now_frame.mesh_id = esp_now_mesh_id;
    esp_now_frame.can_id = msg->identifier;
    esp_now_frame.dlc = msg->data_length_code;
    memcpy(esp_now_frame.data, msg->data, msg->data_length_code);
    err = esp_now_send(broadcast_addr, (uint8_t*)&esp_now_frame,
                       sizeof(esp_now_frame));
    if (err != ESP_OK) {
        debugln("Failed to broadcast CAN msg");
    }
}

twai_message_t message;

// status variables
static bool error_flag = false;
static bool msg_received_flag = false;

// functions
bool can_setup(void);
bool esp_now_setup(void);
int get_mesh_id(void);

// timer
#define TIMER_DIVIDER (16)
#define TIMER_SCALE   (TIMER_BASE_CLK / TIMER_DIVIDER)

static timer_group_t group = TIMER_GROUP_0;
static timer_idx_t timer = TIMER_0;

static bool IRAM_ATTR timer_callback(void* args) {
    if (!error_flag) {
        // No new errors for 1 second, turn off LED
        digitalWrite(GPIO_ERROR_LED, LOW);
        timer_pause(group, timer);  // Stop timer until next error
    } else {
        // Reset flag for next cycle, but keep timer running
        error_flag = false;
    }

    // Reset counter for next cycle
    timer_set_counter_value(group, timer, 0);
    return true;
}

// tasks
void esp_now_task(void* parameters);
void can_task(void* parameters);

// START SETUP

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif

    pinMode(GPIO_ERROR_LED, OUTPUT);

    digitalWrite(GPIO_ERROR_LED, HIGH);

    if (!can_setup()) {
        debugln("Failed to init CAN");
        delay(60 * 1000);
        ESP.restart();
    }

    if (!esp_now_setup()) {
        debugln("Failed to init esp_now");
        delay(60 * 1000);
        ESP.restart();
    }
    esp_now_mesh_id = get_mesh_id();
    debug("Mesh ID: ");
    debugln(esp_now_mesh_id);

    // START TIMER
    // APB_CLK = 80MHz
    timer_config_t timer_config = {};
    timer_config.divider = TIMER_DIVIDER;
    timer_config.counter_dir = TIMER_COUNT_UP;
    timer_config.counter_en = TIMER_PAUSE;
    timer_config.auto_reload = TIMER_AUTORELOAD_EN;
    timer_config.alarm_en = TIMER_ALARM_EN;

    timer_init(group, timer, &timer_config);
    timer_set_counter_value(group, timer, 0);

    uint64_t alarm_value = ERROR_TIMEOUT_MS * (TIMER_SCALE / 1000);
    timer_set_alarm_value(group, timer, alarm_value);
    timer_enable_intr(group, timer);
    timer_isr_callback_add(group, timer, timer_callback, NULL, 0);
    // END TIMER

    xTaskCreate(can_task, "CAN Task", 4096, NULL, 1, NULL);
    xTaskCreate(esp_now_task, "ESP NOW Task", 4096, NULL, 2, NULL);

    delay(1000);
    digitalWrite(GPIO_ERROR_LED, LOW);
}

// END SETUP

// EMPTY LOOP
void loop() {
}

// tasks
void esp_now_task(void* parameters) {
    for (;;) {
        esp_now_send_can_msg(&message);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void can_task(void* parameters) {
    for (;;) {
        uint32_t alerts_triggered;
        twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));

        twai_status_info_t twai_status;

        if (alerts_triggered != 0) {
            twai_get_status_info(&twai_status);
            debugln();
            debugln("CAN STATE:");
            debug("\t");
            switch (twai_status.state) {
                case TWAI_STATE_STOPPED: {
                    debugln("TWAI_STATE_STOPPED");
                } break;
                case TWAI_STATE_RUNNING: {
                    debugln("TWAI_STATE_RUNNING");
                } break;
                case TWAI_STATE_BUS_OFF: {
                    debugln("TWAI_STATE_BUS_OFF");
                } break;
                case TWAI_STATE_RECOVERING: {
                    debugln("TWAI_STATE_RECOVERING");
                } break;
                default:
                    break;
            }

            // enabled alerts:
            // - TWAI_ALERT_RX_DATA
            // - TWAI_ALERT_ERR_PASS
            // - TWAI_ALERT_BUS_ERROR
            // - TWAI_ALERT_RX_QUEUE_FULL
            // - TWAI_ALERT_BUS_OFF
            // - TWAI_ALERT_ABOVE_ERR_WARN
            // - TWAI_ALERT_RX_FIFO_OVERRUN
            if ((alerts_triggered & TWAI_ALERT_RX_DATA) != 0) {
                debugln("CAN OK: MSG RECV");
                while (twai_receive(&message, 0) == ESP_OK) {
                }
                msg_received_flag = true;
            }

            if ((alerts_triggered & TWAI_ALERT_ERR_PASS) != 0) {
                debugln("Alert: TWAI controller has become error passive.");
                error_flag = true;
                debugln("CAN ERROR: ERR PASS");
            }

            if ((alerts_triggered & TWAI_ALERT_BUS_ERROR) != 0) {
                debugln(
                    "Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred "
                    "on "
                    "the bus.");
                debugf("Bus error count: %d\n", twai_status.bus_error_count);
                debugln();
                error_flag = true;
                debugln("CAN ERROR: BUS ERROR");
            }

            if ((alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) != 0) {
                debugln(
                    "Alert: The RX queue is full causing a received frame to "
                    "be "
                    "lost.");
                debugf("\tRX buffered: %d", twai_status.msgs_to_rx);
                debugln();
                debugf("\tRX missed: %d", twai_status.rx_missed_count);
                debugln();
                debugf("\tRX overrun %d", twai_status.rx_overrun_count);
                debugln();
                error_flag = true;
                debugln("CAN ERROR: RX QUEUE FULL");
            }

            if ((alerts_triggered & TWAI_ALERT_BUS_OFF) != 0) {
                debugln(
                    "Alert: Bus-off condition occurred. TWAI controller can no "
                    "longer influence bus");
                error_flag = true;
                debugln("CAN ERROR: BUS OFF");
            }

            if ((alerts_triggered & TWAI_ALERT_ABOVE_ERR_WARN) != 0) {
                debugln(
                    "Alert: One of the error counters have exceeded the error "
                    "warning limit");
                error_flag = true;
                debugln("CAN ERROR: ABOVE ERR WARNING LIMIT");
            }

            if ((alerts_triggered & TWAI_ALERT_RX_FIFO_OVERRUN) != 0) {
                debugln("Alert: An RX FIFO overrun has occurred");
                error_flag = true;
                debugln("CAN ERROR: RX FIFO OVERRUN");
            }
        }
        // blink LED if needed
        if (error_flag) {
            digitalWrite(GPIO_ERROR_LED, HIGH);
            timer_start(group, timer);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool can_setup() {
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(GPIO_CAN_TX, GPIO_CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        debugln("Driver installed\n");
    } else {
        debugln("Failed to install driver\n");
        return false;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        debugln("Driver started\n");
    } else {
        debugln("Failed to start driver\n");
        return false;
    }

    // Reconfigure alerts to detect Error Passive and Bus-Off error states
    uint32_t alerts_to_enable =
        TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_DATA |
        TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_BUS_ERROR |
        TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_RX_FIFO_OVERRUN;

    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        printf("Alerts reconfigured\n");
    } else {
        printf("Failed to reconfigure alerts");
        return false;
    }
    return true;
}

bool esp_now_setup() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        debugln("Error initializing ESP-NOW");
        return false;
    }

    memcpy(peer_info.peer_addr, broadcast_addr, 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;
    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        debugln("Failed to add peer");
        return false;
    }
    return true;
}

int get_mesh_id() {
    uint8_t mac_address[6];
    WiFi.macAddress(mac_address);
    uint32_t uuid = 0;
    for (int i = 2; i < 6; i++) {
        uuid <<= 8;
        uuid |= mac_address[i];
    }
    // Limit the integer to the maximum value of an int (32,767)
    uuid %= 32768;
    return (int)uuid;
}