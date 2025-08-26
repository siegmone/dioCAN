#include "main.hpp"

#include "can.hpp"
#include "transmit.hpp"

// twai variables
static QueueHandle_t can_rx_queue = NULL;

// status variables
static bool error_flag = false;
static bool msg_received_flag = false;

// functions
int get_mesh_id(void);

// timeouts
#define ERROR_TIMEOUT_MS  500
#define SLEEP_TIMEOUT_MS  1000
#define SLEEP_DURATION_MS 10000

// timer
#define TIMER_DIVIDER (16)
#define TIMER_SCALE   (TIMER_BASE_CLK / TIMER_DIVIDER)

static timer_group_t timer_group = TIMER_GROUP_0;
static timer_idx_t error_timer = TIMER_0;
static timer_idx_t sleep_timer = TIMER_1;

static bool IRAM_ATTR error_timer_callback(void* args) {
    if (!error_flag) {
        // No new errors for 1 second, turn off LED
        digitalWrite(GPIO_ERROR_LED, LOW);
        // Stop timer until next error
        timer_pause(timer_group, error_timer);
    }

    error_flag = false;
    // Reset counter for next cycle
    timer_set_counter_value(timer_group, error_timer, 0);
    return true;
}

static bool IRAM_ATTR sleep_timer_callback(void* args) {
    debugln("Going to sleep");
    timer_set_counter_value(timer_group, sleep_timer, 0);
    if (!msg_received_flag) {
        timer_pause(timer_group, sleep_timer);
        esp_deep_sleep(SLEEP_DURATION_MS * 1000);
    }
    return true;
}

static void timer_setup(timer_group_t group, timer_idx_t timer, timer_config_t* config,
                        timer_isr_t callback, uint32_t milliseconds) {
    timer_init(group, timer, config);
    timer_set_counter_value(group, timer, 0);

    float seconds = milliseconds * 0.001;
    uint64_t alarm_value = seconds * (TIMER_SCALE);
    timer_set_alarm_value(group, timer, alarm_value);
    timer_enable_intr(group, timer);
    timer_isr_callback_add(group, timer, callback, NULL, 0);
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

    _esp_now_mesh_id = get_mesh_id();
    debug("Mesh ID: ");
    debugln(esp_now_mesh_id);

    // APB_CLK = 80MHz
    timer_config_t timer_config = {};
    timer_config.divider = TIMER_DIVIDER;
    timer_config.counter_dir = TIMER_COUNT_UP;
    timer_config.counter_en = TIMER_PAUSE;
    timer_config.auto_reload = TIMER_AUTORELOAD_EN;
    timer_config.alarm_en = TIMER_ALARM_EN;

    timer_setup(timer_group, error_timer, &timer_config, error_timer_callback,
                ERROR_TIMEOUT_MS);
    timer_setup(timer_group, sleep_timer, &timer_config, sleep_timer_callback,
                SLEEP_TIMEOUT_MS);

    can_rx_queue = xQueueCreate(64, sizeof(esp_now_packet_t));

    xTaskCreatePinnedToCore(can_task, "CAN Task", 4096, NULL, 1, &can_task_handle, 0);
    xTaskCreatePinnedToCore(esp_now_task, "ESP NOW Task", 4096, NULL, 2,
                            &esp_now_task_handle, 1);

    debugln("System started");

    delay(500);
    digitalWrite(GPIO_ERROR_LED, LOW);
}

// END SETUP

// EMPTY LOOP
void loop() {
}

// tasks
void esp_now_task(void* parameters) {
    TickType_t last_wake_time = xTaskGetTickCount();

    esp_now_packet_t pkt;
    esp_now_batch_t batch;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        batch.count = 0;

        if (xQueueReceive(can_rx_queue, &pkt, portMAX_DELAY) == pdTRUE) {
            batch.msgs[batch.count++] = pkt;
        }

        TickType_t start_time = xTaskGetTickCount();
        while (batch.count < MAX_BATCH_SIZE &&
               xQueueReceive(can_rx_queue, &pkt, pdMS_TO_TICKS(1)) == pdTRUE) {
            batch.msgs[batch.count++] = pkt;
            if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(1))
                break;
        }

        esp_now_send_batch(&batch);

        // xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(ESP_NOW_TASK_DELAY));
    }
}

void can_task(void* parameters) {
    TickType_t last_wake_time = xTaskGetTickCount();

    twai_message_t rx_message;
    uint32_t alerts_triggered;

    timer_start(timer_group, sleep_timer);

    for (;;) {
        msg_received_flag = false;

        twai_read_alerts(&alerts_triggered, portMAX_DELAY);

        if (alerts_triggered != 0) {
            // enabled alerts:
            // - TWAI_ALERT_RX_DATA
            // - TWAI_ALERT_ERR_PASS
            // - TWAI_ALERT_BUS_ERROR
            // - TWAI_ALERT_RX_QUEUE_FULL
            // - TWAI_ALERT_BUS_OFF
            // - TWAI_ALERT_ABOVE_ERR_WARN
            // - TWAI_ALERT_RX_FIFO_OVERRUN

            if ((alerts_triggered & TWAI_ALERT_RX_DATA) != 0) {
                debugln("CAN RECV.......OK");
                while (twai_receive(&rx_message, 0) == ESP_OK) {
                    debugf("MSG............%03X  [%d]  ", rx_message.identifier,
                           rx_message.data_length_code);
                    debugln();

                    esp_now_packet_t pkt = {0};

                    uint64_t current_time = esp_timer_get_time();

                    memcpy(&pkt.data, &rx_message.data, rx_message.data_length_code);
                    pkt.can_id = rx_message.identifier;
                    pkt.dlc = rx_message.data_length_code;
                    pkt.timestamp = current_time;
                    pkt.mesh_id = _esp_now_mesh_id;

                    if (xQueueSend(can_rx_queue, &pkt, 0) != pdTRUE) {
                        debugln("Failed to post to Queue");
                    }

                    msg_received_flag = true;
                }

                if (esp_now_task_handle) {
                    xTaskNotifyGive(esp_now_task_handle);
                }
            }

            if ((alerts_triggered & TWAI_ALERT_ERR_PASS) != 0) {
                debugln("Alert: TWAI controller has become error passive.");
                error_flag = true;
                debugln("CAN ERROR......ERR PASS");
            }

            if ((alerts_triggered & TWAI_ALERT_BUS_ERROR) != 0) {
                debugln(
                    "Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred "
                    "on "
                    "the bus.");
                debugf("Bus error count: %d\n", twai_status.bus_error_count);
                debugln();
                error_flag = true;
                debugln("CAN ERROR......BUS ERROR");
            }

            if ((alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) != 0) {
                debugln(
                    "Alert: The RX queue is full causing a received frame to "
                    "be "
                    "lost.");
                debugf("....RX buffered: %d", twai_status.msgs_to_rx);
                debugln();
                debugf("....RX missed: %d", twai_status.rx_missed_count);
                debugln();
                debugf("....RX overrun %d", twai_status.rx_overrun_count);
                debugln();
                error_flag = true;
                debugln("CAN ERROR......RX QUEUE FULL");
            }

            if ((alerts_triggered & TWAI_ALERT_BUS_OFF) != 0) {
                debugln(
                    "Alert: Bus-off condition occurred. TWAI controller can no "
                    "longer influence bus");
                error_flag = true;
                debugln("CAN ERROR......BUS OFF");
            }

            if ((alerts_triggered & TWAI_ALERT_ABOVE_ERR_WARN) != 0) {
                debugln(
                    "Alert: One of the error counters have exceeded the error "
                    "warning limit");
                error_flag = true;
                debugln("CAN ERROR......ABOVE ERR WARNING LIMIT");
            }

            if ((alerts_triggered & TWAI_ALERT_RX_FIFO_OVERRUN) != 0) {
                debugln("Alert: An RX FIFO overrun has occurred");
                error_flag = true;
                debugln("CAN ERROR......RX FIFO OVERRUN");
            }
        }

        // check for error and light the led
        if (error_flag) {
            digitalWrite(GPIO_ERROR_LED, HIGH);
            timer_start(timer_group, error_timer);
        }

        if (msg_received_flag) {
            timer_set_counter_value(timer_group, sleep_timer, 0);
        }

        can_check_status();
    }
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
