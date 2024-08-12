/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "rc.hpp"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "flight_control.hpp"

// esp_now_peer_info_t slave;

volatile uint16_t Connect_flag = 0;

// Telemetry相手のMAC ADDRESS 4C:75:25:AD:B6:6C
// ATOM Lite (C): 4C:75:25:AE:27:FC
// 4C:75:25:AD:8B:20
// 4C:75:25:AF:4E:84
// 4C:75:25:AD:8B:20
// 4C:75:25:AD:8B:20 赤水玉テープ　ATOM lite
uint8_t TelemAddr[6] = {0};
// uint8_t TelemAddr[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
volatile uint8_t MyMacAddr[6];
volatile uint8_t peer_command[4] = {0xaa, 0x55, 0x16, 0x88};
volatile uint8_t Rc_err_flag     = 0;
esp_now_peer_info_t peerInfo;

// RC
volatile float Stick[16];
volatile uint8_t Recv_MAC[3];

void on_esp_now_sent(const uint8_t *mac_addr, esp_now_send_status_t status);

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len) {
    Connect_flag = 0;

    uint8_t *d_int;
    // int16_t d_short;
    float d_float;

    if (!TelemAddr[0] && !TelemAddr[1] && !TelemAddr[2] && !TelemAddr[3] && !TelemAddr[4] && !TelemAddr[5]) {
        memcpy(TelemAddr, mac_addr, 6);
        memcpy(peerInfo.peer_addr, TelemAddr, 6);
        peerInfo.channel = CHANNEL;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            USBSerial.println("Failed to add peer2");
            memset(TelemAddr, 0, 6);
        } else {
            esp_now_register_send_cb(on_esp_now_sent);
        }
    }

    Recv_MAC[0] = recv_data[0];
    Recv_MAC[1] = recv_data[1];
    Recv_MAC[2] = recv_data[2];

    if ((recv_data[0] == MyMacAddr[3]) && (recv_data[1] == MyMacAddr[4]) && (recv_data[2] == MyMacAddr[5])) {
        Rc_err_flag = 0;
    } else {
        Rc_err_flag = 1;
        return;
    }

    // checksum
    uint8_t check_sum = 0;
    for (uint8_t i = 0; i < 24; i++) check_sum = check_sum + recv_data[i];
    // if (check_sum!=recv_data[23])USBSerial.printf("checksum=%03d recv_sum=%03d\n\r", check_sum, recv_data[23]);
    if (check_sum != recv_data[24]) {
        Rc_err_flag = 1;
        return;
    }

    d_int         = (uint8_t *)&d_float;
    d_int[0]      = recv_data[3];
    d_int[1]      = recv_data[4];
    d_int[2]      = recv_data[5];
    d_int[3]      = recv_data[6];
    Stick[RUDDER] = d_float;

    d_int[0]        = recv_data[7];
    d_int[1]        = recv_data[8];
    d_int[2]        = recv_data[9];
    d_int[3]        = recv_data[10];
    Stick[THROTTLE] = d_float;

    d_int[0]       = recv_data[11];
    d_int[1]       = recv_data[12];
    d_int[2]       = recv_data[13];
    d_int[3]       = recv_data[14];
    Stick[AILERON] = d_float;

    d_int[0]        = recv_data[15];
    d_int[1]        = recv_data[16];
    d_int[2]        = recv_data[17];
    d_int[3]        = recv_data[18];
    Stick[ELEVATOR] = d_float;

    Stick[BUTTON_ARM]     = recv_data[19];  // auto_up_down_status
    Stick[BUTTON_FLIP]    = recv_data[20];
    Stick[CONTROLMODE]    = recv_data[21];  // Mode:rate or angle control
    Stick[ALTCONTROLMODE] = recv_data[22];  // 高度制御

    ahrs_reset_flag = recv_data[23];

    Stick[LOG] = 0.0;
    // if (check_sum!=recv_data[23])USBSerial.printf("checksum=%03d recv_sum=%03d\n\r", check_sum, recv_data[23]);

#if 0
  USBSerial.printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f  %6.3f\n\r", 
                                            Stick[THROTTLE],
                                            Stick[AILERON],
                                            Stick[ELEVATOR],
                                            Stick[RUDDER],
                                            Stick[BUTTON_ARM],
                                            Stick[BUTTON_FLIP],
                                            Stick[CONTROLMODE],
                                            Stick[ALTCONTROLMODE],
                                            Stick[LOG]);
#endif
}

// 送信コールバック
uint8_t esp_now_send_status;
void on_esp_now_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    esp_now_send_status = status;
}

void rc_init(void) {
    // Initialize Stick list
    for (uint8_t i = 0; i < 16; i++) Stick[i] = 0.0;

    // ESP-NOW初期化
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    WiFi.macAddress((uint8_t *)MyMacAddr);
    USBSerial.printf("MAC ADDRESS: %02X:%02X:%02X:%02X:%02X:%02X\r\n", MyMacAddr[0], MyMacAddr[1], MyMacAddr[2],
                     MyMacAddr[3], MyMacAddr[4], MyMacAddr[5]);

    if (esp_now_init() == ESP_OK) {
        USBSerial.println("ESPNow Init Success");
    } else {
        USBSerial.println("ESPNow Init Failed");
        ESP.restart();
    }

    // MACアドレスブロードキャスト
    uint8_t addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(peerInfo.peer_addr, addr, 6);
    peerInfo.channel = CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        USBSerial.println("Failed to add peer");
        return;
    }
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Send my MAC address
    for (uint16_t i = 0; i < 50; i++) {
        send_peer_info();
        delay(50);
        USBSerial.printf("%d\n", i);
    }

    // ESP-NOW再初期化
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
        USBSerial.println("ESPNow Init Success2");
    } else {
        USBSerial.println("ESPNow Init Failed2");
        ESP.restart();
    }

    // ESP-NOWコールバック登録
    esp_now_register_recv_cb(OnDataRecv);
    USBSerial.println("ESP-NOW Ready.");
}

void send_peer_info(void) {
    uint8_t data[11];
    data[0] = CHANNEL;
    memcpy(&data[1], (uint8_t *)MyMacAddr, 6);
    memcpy(&data[1 + 6], (uint8_t *)peer_command, 4);
    esp_now_send(peerInfo.peer_addr, data, 11);
}

uint8_t telemetry_send(uint8_t *data, uint16_t datalen) {
    static uint32_t cnt       = 0;
    static uint8_t error_flag = 0;
    static uint8_t state      = 0;

    esp_err_t result;

    if ((error_flag == 0) && (state == 0)) {
        result = esp_now_send(peerInfo.peer_addr, data, datalen);
        cnt    = 0;
    } else
        cnt++;

    if (esp_now_send_status == 0) {
        error_flag = 0;
        // state = 0;
    } else {
        error_flag = 1;
        // state = 1;
    }
    // 一度送信エラーを検知してもしばらくしたら復帰する
    if (cnt > 500) {
        error_flag = 0;
        cnt        = 0;
    }
    cnt++;
    // USBSerial.printf("%6d %d %d\r\n", cnt, error_flag, esp_now_send_status);

    return error_flag;
}

void rc_end(void) {
    // Ps3.end();
}

uint8_t rc_isconnected(void) {
    bool status;
    Connect_flag++;
    if (Connect_flag < 40)
        status = 1;
    else
        status = 0;
    // USBSerial.printf("%d \n\r", Connect_flag);
    return status;
}

void rc_demo() {
}
