// Decomment to display debug informations
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

// Decomment to enable the use of an i2c oled display
#define DISPLAY_ATTACHED

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include <algorithm>
#include <vector>

#include "driver/gpio.h"
#include "driver/twai.h"

#ifdef DISPLAY_ATTACHED
// for display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define SCREEN_WIDTH  128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64   // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
void display_start(void);
void display_end(void);
void display_center(String text);

uint32_t last_check_fps_timestamp = 0;
uint32_t frames_per_second_display = 0;
#endif  // DISPLAY_ATTACHED

// duration esp32 has to wait before going to sleep after no CAN avitivites in
// (in milliseconds)
#define CAN_IDLE_TIMEOUT 800
// duration ESP32 will sleep for (in seconds)
#define SLEEP_PERIOD_SEC 5
#define POLLING_RATE_MS  100
#define ERROR_TIMEOUT    500
#define GRACE_PERIOD     CAN_IDLE_TIMEOUT

// Pin definitions

#define GPIO_ERROR_LED GPIO_NUM_17
#define GPIO_SLEEP_LED GPIO_NUM_5
#define GPIO_CAN_TX    GPIO_NUM_18
#define GPIO_CAN_RX    GPIO_NUM_19

// Functions

bool can_setup(void);
bool esp_now_setup(void);
int get_mesh_id(void);

// Global variables

uint32_t current_millis;
uint32_t last_can_msg_timestamp = 0;
uint32_t last_error_timestamp = 0;
uint32_t wakeup_timestamp = 0;
uint32_t frames_per_second = 0;

// ESP-NOW communication

typedef struct esp_now_frame_s {
    int mesh_id;
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} esp_now_frame_t;

esp_now_frame_t esp_now_frame;

uint8_t broadcast_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peer_info;
int esp_now_mesh_id;

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

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif

    pinMode(GPIO_ERROR_LED, OUTPUT);
    pinMode(GPIO_SLEEP_LED, OUTPUT);

    last_can_msg_timestamp = millis() - CAN_IDLE_TIMEOUT + 5;

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
    debugln(esp_now_mesh_id);

#ifdef DISPLAY_ATTACHED
    // Address 0x3D for 128x64. Changed D to C
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        debugln(F("SSD1306 allocation failed"));
        delay(60 * 1000);
        ESP.restart();
    }
#endif

    wakeup_timestamp = millis();

    digitalWrite(GPIO_ERROR_LED, LOW);
    digitalWrite(GPIO_SLEEP_LED, HIGH);

    delay(1000);
}

void loop() {
    // Variable to determine if LED should blink
    static bool error_led_on = false;
    current_millis = millis();

    twai_message_t message;
    bool msg_received = false;
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));

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
            while (twai_receive(&message, 0) == ESP_OK) {
                debug("New CAN ID detected: ");
                debugf("%x", message.identifier);
                debugln();
                esp_now_send_can_msg(&message);
                frames_per_second++;
            }
            last_can_msg_timestamp = millis();
            msg_received = true;
        }

        if ((alerts_triggered & TWAI_ALERT_ERR_PASS) != 0) {
            debugln("Alert: TWAI controller has become error passive.");
            error_led_on = true;
            debugln(" CAN MSG..........ERROR");
        }

        if ((alerts_triggered & TWAI_ALERT_BUS_ERROR) != 0) {
            debugln(
                "Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on "
                "the bus.");
            // debugf("Bus error count: %d\n",
            // twaistatus.bus_error_count);
            error_led_on = true;
            debugln(" CAN MSG......BUS ERROR");
        }

        if ((alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) != 0) {
            debugln(
                "Alert: The RX queue is full causing a received frame to be "
                "lost.");
            // debugf("RX buffered: %d\t", twaistatus.msgs_to_rx);
            // debugf("RX missed: %d\t", twaistatus.rx_missed_count);
            // debugf("RX overrun %d\n", twaistatus.rx_overrun_count);
            error_led_on = true;
            debugln(" CAN MSG.........Q FULL");
        }

        if ((alerts_triggered & TWAI_ALERT_BUS_OFF) != 0) {
            debugln(
                "Alert: Bus-off condition occurred. TWAI controller can no "
                "longer influence bus");
            error_led_on = true;
            debugln(" CAN MSG........BUS OFF");
        }

        if ((alerts_triggered & TWAI_ALERT_ABOVE_ERR_WARN) != 0) {
            debugln("Alert: One of the error counters have exceeded the error warning limit");
            error_led_on = true;
            debugln(" CAN MSG.......WARN ERR");
        }

        if ((alerts_triggered & TWAI_ALERT_RX_FIFO_OVERRUN) != 0) {
            debugln("Alert: An RX FIFO overrun has occurred");
            error_led_on = true;
            debugln(" CAN MSG........FIFO OR");
        }

        // blink LED if needed
        if (error_led_on) {
            digitalWrite(GPIO_ERROR_LED, HIGH);
            last_error_timestamp = current_millis;
        }
    }

    if (current_millis - last_error_timestamp >= ERROR_TIMEOUT) {
        digitalWrite(GPIO_ERROR_LED, LOW);
        last_error_timestamp = 0;
        error_led_on = false;
    }

    if (current_millis - last_can_msg_timestamp > CAN_IDLE_TIMEOUT &&
        !msg_received && (current_millis - wakeup_timestamp > GRACE_PERIOD)) {
#ifdef DISPLAY_ATTACHED
        display_start();
        display.setTextSize(4);
        {
            display_center(String("SLEEP"));
        }
        display_end();
#endif  // DISPLAY_ATTACHED
        digitalWrite(GPIO_SLEEP_LED, LOW);
        esp_deep_sleep(1e6 * SLEEP_PERIOD_SEC);
        return;
    }

    // Every reset timer every second
    // TODO(siegmone): Should actually implement a real timer idk if the esp has
    // one
    if (current_millis - last_check_fps_timestamp > 1000) {
        debugf("CAN-FPS: %2d", frames_per_second);
        debugln();
        last_check_fps_timestamp = current_millis;
        frames_per_second_display = frames_per_second;
        frames_per_second = 0;
    }

#ifdef DISPLAY_ATTACHED

    display_start();
    {
        if (error_led_on) {
            display.setTextSize(4);
            display_center(String("ERROR"));
        } else {
            display.setTextSize(4);
            display_center(String(frames_per_second_display));
        }
    }
    display_end();
#endif  // DISPLAY_ATTACHED
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

#ifdef DISPLAY_ATTACHED
void display_start() {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
}

void display_end() {
    display.display();
}

void display_center(String text) {
    int16_t x1;
    int16_t y1;
    uint16_t width;
    uint16_t height;

    display.getTextBounds(text, 0, 0, &x1, &y1, &width, &height);

    // display on horizontal and vertical center
    display.setCursor((SCREEN_WIDTH - width) / 2, (SCREEN_HEIGHT - height) / 2);
    display.println(text);  // text to display
}
#endif  // DISPLAY_ATTACHED