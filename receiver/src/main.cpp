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

#include <unordered_map>

#define EMPTY_MAP_TIME 30

uint32_t last_empty_timestamp = 0;

// SavvyCAN serial variables

#define SERIAL_MAX_BUFF_LEN 30
char slcan_com[SERIAL_MAX_BUFF_LEN];
bool send_ok = true;
bool send_can_msgs = true;
bool send_timestamp = true;

// ESP-NOW definitions

#define MESH_ID 18460

typedef struct esp_now_frame_s {
    int mesh_id;
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} esp_now_frame_t;

// ESP-NOW recv callback
void recv_cb(const uint8_t *mac, const uint8_t *data, int len);

// SavvyCAN functions
void send_can_over_serial(const esp_now_frame_t *ef);
void handle_serial(void);
void send_ack(void);
void send_nack(void);
unsigned short slcan_get_time(void);

// map that checks which msg has been recvd
static std::unordered_map<uint32_t, bool> map_recv;
// map that holds the most recent can_messages
static std::unordered_map<uint32_t, esp_now_frame_t> map_msgs;

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    while (esp_now_init() != ESP_OK) {
        debugln("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(recv_cb);

    delay(500);
}

void loop() {
    uint32_t current_millis = millis();

    if (map_msgs.empty()) {
        return;
    }

    for (const auto &recv_pair : map_recv) {
        if (recv_pair.second) {
            esp_now_frame_t ef = map_msgs[recv_pair.first];
            send_can_over_serial(&ef);
            handle_serial();
        }
        map_recv[recv_pair.first] = false;
    }

    if (current_millis - last_empty_timestamp > EMPTY_MAP_TIME * 1000) {
        map_recv.clear();
        map_msgs.clear();
        last_empty_timestamp = current_millis;
    }
}

// esp_now recv callback
void recv_cb(const uint8_t *mac, const uint8_t *data, int len) {
    esp_now_frame_t esp_now_frame;
    memcpy(&esp_now_frame, data, sizeof(esp_now_frame));
    map_msgs[esp_now_frame.can_id] = esp_now_frame;
    map_recv[esp_now_frame.can_id] = true;
}

// SavvyCAN functions
void send_can_over_serial(const esp_now_frame_t *ef) {
    // total_buf_size:
    // + can_id(9)
    // + dlc(1)
    // + data(16)
    // + timestamp(4)
    // + carriage_return(1)
    // + null_byte(1)
    // = 32
    char buf[36];
    int idx = 0;

    if (ef->can_id <= 0x7FF) {
        // Standard 11-bit CAN ID
        idx += snprintf(buf + idx, sizeof(buf) - idx, "t%03x", ef->can_id);
    } else if (ef->can_id <= 0x1FFFFFFF) {
        // Extended 29-bit CAN ID
        idx += snprintf(buf + idx, sizeof(buf) - idx, "T%08x", ef->can_id);
    } else {
        // Invalid CAN ID
        return;
    }

    idx += snprintf(buf + idx, sizeof(buf) - idx, "%d", ef->dlc);
    for (int i = 0; i < ef->dlc; i++) {
        idx += snprintf(buf + idx, sizeof(buf) - idx, "%02x", ef->data[i]);
    }

    if (send_timestamp) {
        idx += snprintf(buf + idx, sizeof(buf) - idx, "%02x", slcan_get_time());
    }

    buf[idx++] = '\r';  // Carriage return
    buf[idx++] = '\0';  // Null-terminate the buffer

    // Send it through the serial
    Serial.print(buf);
}

void handle_serial() {
    // clang-format off
    /* ============================ SLCAN/LAWICEL Commands ====================================

            ACK = Acknoledged but not implemented ( OK returned)
            Yes = implemented
                -  = Not implemented


        CMD | IMPLEMENTED | SYNTAX               | DESCRIPTION
        ------------------------------------------------------------------------------------------------------------
        'S' |   ACK       |   Sn[CR]               The CAN Gateway cannot be changed over this tool
            |             |                        Setup with standard CAN bit-rates where n is 0-8.
            |             |                        S0 10Kbit,S4 125Kbit, S8 1Mbit, S1 20Kbit, S5 250Kbit, S9 83.3Kbit, S2 50Kbit, S6 500Kbit, S3 100Kbit, S7 800Kbit
        's' |   ACK       |   sxxyy[CR]            Setup with BTR0/BTR1 CAN bit-rates where xx and yy is a hex value.
        'O' |   ACK       |   O[CR]                Open the CAN channel in normal mode (sending & receiving).
        'L' |   ACK       |   L[CR]                Open the CAN channel in listen only mode (receiving).
        'C' |   YES       |   C[CR]                Close the CAN channel.
        't' |   YES       |   tiiildd...[CR]       Transmit a standard (11bit) CAN frame.
        'T' |   ACK       |   Tiiiiiiiildd...[CR]  Transmit an extended (29bit) CAN frame
        'r' |   ACK       |   riiil[CR]            Transmit an standard RTR (11bit) CAN frame.
        'R' |   ACK       |   Riiiiiiiil[CR]       Transmit an extended RTR (29bit) CAN frame.
        'P' |   ACK       |   P[CR]                Poll incomming FIFO for CAN frames (single poll)
        'A' |   ACK       |   A[CR]                Polls incomming FIFO for CAN frames (all pending frames)
        'F' |   ACK       |   F[CR]                Read Status Flags: bit 0 = RX Fifo Full, 1 = TX Fifo Full, 2 = Error warning, 3 = Data overrun, 5= Error passive, 6 = Arb. Lost, 7 = Bus Error
        'X' |   ACK       |   Xn[CR]               Sets Auto Poll/Send ON/OFF for received frames.
        'W' |    -        |   Wn[CR]               Filter mode setting. By default CAN232 works in dual filter mode (0) and is backwards compatible with previous CAN232 versions.
        'M' |    -        |   Mxxxxxxxx[CR]        Sets Acceptance Code Register (ACn Register of SJA1000). // we use MCP2515, not supported
        'm' |    -        |   mxxxxxxxx[CR]        Sets Acceptance Mask Register (AMn Register of SJA1000). // we use MCP2515, not supported
        'U' |   ACK       |   Un[CR]               Setup new UART baud rate n=0-6: 0-230400, 1:115200, 2:57600, 3:38400, 4:19200, 5:9600, 6:2400
        'V' |   YES       |   v[CR]                Get Version number of both CAN232 hardware and software
        'v' |   YES       |   V[CR]                Get Version number of both CAN232 hardware and software
        'N' |   YES       |   N[CR]                Get Serial number of the CAN232.
        'Z' |   YES       |   Zn[CR]               Sets Time Stamp ON/OFF for received frames only. EXTENSION to LAWICEL: Z2 - millis() timestamp w/o standard 60000ms cycle
        'Q' |    -        |   Qn[CR]               Auto Startup feature (from power on).

    */
    // clang-format on

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

unsigned short slcan_get_time() {
    unsigned long current_time = millis();
    unsigned short timestamp =
        static_cast<unsigned short>(current_time % 60000);
    return timestamp;
}