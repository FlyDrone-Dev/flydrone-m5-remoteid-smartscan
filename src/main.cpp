/*
 * SPDX-FileCopyrightText: 2025 Braveridge co., ltd.
 * SPDX-License-Identifier: MIT
 *
 * Remote ID receiver firmware for M5StickC Plus2
 * Ported from M5Stamp S3 (bv-remote-id)
 */

#include <Arduino.h>
#include <M5StickCPlus2.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "config.h"
#include "ble/BLEManager.h"
#include "wifi/WifiData.h"
#include "drone/DroneTracker.h"
#include "logo_data.h"

// ---------- Display modes ---------------------------------------------------

enum DisplayMode : uint8_t {
    MODE_LIST = 0,  // Drone count, last serial, channel, battery
    MODE_DETAIL,    // Full details of the last received drone
    MODE_STATS,     // Cumulative statistics
    MODE_COUNT
};

// ---------- Globals ---------------------------------------------------------

static BLEManager    bleManager;
static DroneTracker  droneTracker;
static uint8_t       currentWifiChannel  = WIFI_CHANNEL_MIN;
static DisplayMode   displayMode         = MODE_LIST;
static unsigned long lastDisplayMs       = 0;
static volatile bool displayNeedsUpdate  = true;

// ---------- WiFi promiscuous callback ---------------------------------------

void IRAM_ATTR wifiReceiveCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
    // Hardware filter is set to MGMT, but guard anyway
    if (type != WIFI_PKT_MGMT) return;

    wifi_promiscuous_pkt_t* ppkt = (wifi_promiscuous_pkt_t*)buf;
    WifiMacHeader*    mac         = (WifiMacHeader*)ppkt->payload;

    // Accept beacon (0x80) and probe response (0x50) frames only
    uint8_t fc0 = (uint8_t)(mac->frameControl & 0xFF);
    if (fc0 != 0x80 && fc0 != 0x50) return;

    WiFiElementData* elementData = (WiFiElementData*)mac->payload;
    int len = (int)ppkt->rx_ctrl.sig_len - (int)sizeof(WifiMacHeader);
    if (len <= 0) return;

    while (len > 4) {
        if (elementData->id == 221) { // Vendor Specific IE
            vendor_ie_data_t* vi = (vendor_ie_data_t*)elementData;

            if (vi->length >= 4 && isOuiRemoteId(vi)) {
                int payloadLen    = vi->length - 4; // minus OUI(3) + type(1)
                int notifyDataLen = vi->length + 4;

                // Stack buffer: max vendor IE body = 255 bytes → notify max 259
                uint8_t notifyData[260];
                notifyData[0] = 0x01;                               // notify type for BvRID app
                notifyData[1] = ppkt->rx_ctrl.channel;              // wifi channel
                notifyData[2] = (uint8_t)(int8_t)ppkt->rx_ctrl.rssi; // rssi
                notifyData[3] = vi->length;                         // rid data length
                memcpy(notifyData + 4, vi->vendor_oui, 3);          // vendor OUI
                notifyData[7] = vi->vendor_oui_type;                // OUI type
                if (payloadLen > 0) {
                    memcpy(notifyData + 8, vi->payload, payloadLen);
                }

                printf("\nnotify: ");
                for (int i = 0; i < notifyDataLen; i++) printf("%02X ", notifyData[i]);
                printf("\n");

                // Update drone tracker (parses OpenDroneID payload)
                if (payloadLen > 0) {
                    droneTracker.update(
                        mac->sourceAddress,
                        vi->payload,
                        payloadLen,
                        (int8_t)ppkt->rx_ctrl.rssi,
                        ppkt->rx_ctrl.channel
                    );
                }

                bleManager.notifyRidData(notifyData, notifyDataLen);
                displayNeedsUpdate = true;
            }
        }

        int dlen = 2 + (int)elementData->length;
        if (dlen <= 0 || dlen > len) break; // Malformed IE: stop
        len -= dlen;
        elementData = (WiFiElementData*)((uint8_t*)elementData + dlen);
    }
}

// ---------- WiFi channel rotation -------------------------------------------

static uint8_t getNextWifiChannel() {
    if (++currentWifiChannel > WIFI_CHANNEL_MAX) {
        currentWifiChannel = WIFI_CHANNEL_MIN;
    }
    return currentWifiChannel;
}

// ---------- Display helpers -------------------------------------------------

static void drawHeader() {
    M5.Lcd.fillRect(0, 0, 240, 18, TFT_NAVY);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_NAVY);
    M5.Lcd.setTextSize(1);

    M5.Lcd.setCursor(3, 5);
    M5.Lcd.print("Remote ID");

    // BLE connected indicator
    if (bleManager.isConnected) {
        M5.Lcd.setTextColor(TFT_CYAN, TFT_NAVY);
        M5.Lcd.setCursor(88, 5);
        M5.Lcd.print("BLE");
    }

    // WiFi channel
    M5.Lcd.setTextColor(TFT_WHITE, TFT_NAVY);
    M5.Lcd.setCursor(148, 5);
    M5.Lcd.printf("Ch:%02d", currentWifiChannel);

    // Battery level with colour-coded indicator
    int bat = M5.Power.getBatteryLevel();
    if (bat < 0) bat = 0;
    uint16_t batColor = (bat > 50) ? TFT_GREEN : (bat > 20 ? TFT_YELLOW : TFT_RED);
    M5.Lcd.setTextColor(batColor, TFT_NAVY);
    M5.Lcd.setCursor(196, 5);
    M5.Lcd.printf("%3d%%", bat);
}

static void drawListMode() {
    int      droneCount = droneTracker.getActiveDroneCount();
    DroneInfo last;
    bool      hasLast   = droneTracker.getLastUpdated(last);

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setTextSize(2);

    // Drone count
    M5.Lcd.setCursor(4, 24);
    M5.Lcd.printf("Drones: %-3d", droneCount);

    // Last UAS ID (trailing 8 characters)
    M5.Lcd.setCursor(4, 50);
    if (hasLast && last.hasUasId) {
        int   idLen = strlen(last.uasId);
        const char* tail = (idLen > 8) ? (last.uasId + idLen - 8) : last.uasId;
        M5.Lcd.printf("Last:%-8s", tail);
    } else {
        M5.Lcd.print("Last:--------");
    }

    // RSSI of last received frame
    M5.Lcd.setCursor(4, 76);
    if (hasLast) {
        M5.Lcd.printf("RSSI:% 4ddBm", last.rssi);
    } else {
        M5.Lcd.print("RSSI: ---    ");
    }

    // Button hints
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    M5.Lcd.setCursor(4, 122);
    M5.Lcd.print("[A]Mode:[LIST]  [B]Reset");
}

static void drawDetailMode() {
    DroneInfo last;
    bool hasLast = droneTracker.getLastUpdated(last);

    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.setCursor(4, 22);
    M5.Lcd.print("=== DETAIL ===");

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

    if (hasLast && last.hasUasId) {
        M5.Lcd.setCursor(4, 36);
        M5.Lcd.printf("ID:%.20s", last.uasId);
        M5.Lcd.setCursor(4, 50);
        M5.Lcd.printf("IDtype:%d  UAtype:%d", last.idType, last.uaType);
    } else {
        M5.Lcd.setCursor(4, 36);
        M5.Lcd.print("ID: (no data yet)   ");
    }

    if (hasLast) {
        M5.Lcd.setCursor(4, 64);
        M5.Lcd.printf("Ch:%02d  RSSI:% 4ddBm", last.wifiChannel, last.rssi);

        M5.Lcd.setCursor(4, 78);
        M5.Lcd.printf("MAC:%02X%02X%02X%02X%02X%02X",
            last.macAddress[0], last.macAddress[1], last.macAddress[2],
            last.macAddress[3], last.macAddress[4], last.macAddress[5]);

        M5.Lcd.setCursor(4, 92);
        uint32_t ageSec = (millis() - last.lastSeenMs) / 1000;
        M5.Lcd.printf("Age: %lus ago        ", ageSec);
    }

    M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    M5.Lcd.setCursor(4, 122);
    M5.Lcd.print("[A]Mode:[DETAIL] [B]Reset");
}

static void drawStatsMode() {
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.setCursor(4, 22);
    M5.Lcd.print("=== STATS ===");

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

    M5.Lcd.setCursor(4, 36);
    M5.Lcd.printf("Total rcvd : %lu", droneTracker.getTotalReceived());

    M5.Lcd.setCursor(4, 50);
    M5.Lcd.printf("Active now : %d", droneTracker.getActiveDroneCount());

    M5.Lcd.setCursor(4, 64);
    M5.Lcd.printf("Scan ch    : %02d", currentWifiChannel);

    M5.Lcd.setCursor(4, 78);
    M5.Lcd.printf("BLE        : %s", bleManager.isConnected ? "Connected" : "Waiting");

    M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    M5.Lcd.setCursor(4, 122);
    M5.Lcd.print("[A]Mode:[STATS]  [B]Reset");
}

static void updateDisplay() {
    drawHeader();

    // Clear content area (below header, above button hints area)
    M5.Lcd.fillRect(0, 19, 240, 102, TFT_BLACK);

    switch (displayMode) {
        case MODE_LIST:   drawListMode();   break;
        case MODE_DETAIL: drawDetailMode(); break;
        case MODE_STATS:  drawStatsMode();  break;
        default:          break;
    }
}

// ---------- setup / loop ----------------------------------------------------

void setup() {
    Serial.begin(115200);

    // Initialize M5StickC Plus2 (display, power IC, buttons, IMU)
    M5.begin();

    // --- Logo splash ---
    M5.Lcd.setRotation(3);                                  // 横向き表示（通常画面と同じ向き）
    M5.Lcd.drawJpg(LOGO_JPEG, LOGO_JPEG_SIZE, 0, 0);       // ロゴ全画面表示
    delay(2000);                                             // 2秒待機
    M5.Lcd.fillScreen(BLACK);                               // 画面クリア

    M5.Lcd.setRotation(3);          // Landscape: 240 x 135
    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setBrightness(80);       // Moderate brightness to save battery

    // Splash screen
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(20, 40);
    M5.Lcd.print("Remote ID");
    M5.Lcd.setCursor(20, 65);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.print("Initializing...");

    // WiFi: station mode + promiscuous receive (management frames only)
    WiFi.mode(WIFI_STA);
    wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifiReceiveCallback);

    // BLE
    uint8_t versionData[] = {1, 0, 0};
    bleManager.startBLE(ADV_LOCAL_NAME, versionData);

    updateDisplay();
}

void loop() {
    M5.update(); // Refresh button state

    // Button A: cycle through display modes
    if (M5.BtnA.wasPressed()) {
        displayMode = (DisplayMode)((displayMode + 1) % MODE_COUNT);
        displayNeedsUpdate = true;
    }

    // Button B: clear all tracked drone data
    if (M5.BtnB.wasPressed()) {
        droneTracker.reset();
        displayNeedsUpdate = true;
    }

    // Advance WiFi channel (rotates WIFI_CHANNEL_MIN .. WIFI_CHANNEL_MAX)
    uint8_t ch = getNextWifiChannel();
    ESP_ERROR_CHECK(esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE));
    printf("wifi scan channel : %d\n", ch);

    // Refresh display at most every DISPLAY_UPDATE_INTERVAL ms
    unsigned long now = millis();
    if (displayNeedsUpdate || (now - lastDisplayMs >= DISPLAY_UPDATE_INTERVAL)) {
        updateDisplay();
        lastDisplayMs      = now;
        displayNeedsUpdate = false;
    }

    delay(500);
}
