#include "can.hpp"

#include "main.hpp"


bool can_setup() {
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        GPIO_CAN_TX, GPIO_CAN_RX, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        debugln("CAN INSTALL....OK");
    } else {
        debugln("CAN INSTALL....FAIL");
        return false;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        debugln("CAN STATE......OK");
    } else {
        debugln("CAN STATE......FAIL");
        return false;
    }

    // Reconfigure alerts
    uint32_t alerts_to_enable =
        TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_DATA |
        TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_BUS_ERROR |
        TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_RX_FIFO_OVERRUN;

    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        debugln("Alerts reconfigured\n");
    } else {
        debugln("Failed to reconfigure alerts");
        return false;
    }

    return true;
}

void can_check_status() {
    twai_status_info_t twai_status;
    twai_get_status_info(&twai_status);

    debug("CAN STATE......");
    switch (twai_status.state) {
        case TWAI_STATE_STOPPED: {
            debugln("TWAI_STATE_STOPPED");
        } break;
        case TWAI_STATE_RUNNING: {
            debugln("TWAI_STATE_RUNNING");
        } break;
        case TWAI_STATE_BUS_OFF: {
            debugln("TWAI_STATE_BUS_OFF");
            twai_stop();
            twai_start();
        } break;
        case TWAI_STATE_RECOVERING: {
            debugln("TWAI_STATE_RECOVERING");
        } break;
        default:
            break;
    }
}
