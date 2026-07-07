/*
 * SPDX-FileCopyrightText: 2025 Braveridge co., ltd.
 * SPDX-License-Identifier: MIT
 *
 * CoreS3-SE 移植 事前コンパイル確認用の仮ファイル。
 * 実機未検証。ble/drone/wifi のロジックは main.cpp と共通のものをそのまま利用する。
 */

#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "config.h"
#include "ble/BLEManager.h"
#include "wifi/WifiData.h"
#include "drone/DroneTracker.h"
#include "logo_data_cores3.h"

// ---------- Display modes ---------------------------------------------------

enum DisplayMode : uint8_t {
    MODE_LIST = 0,  // Drone count, last serial, channel, battery
    MODE_DETAIL,    // Full details of the last received drone
    MODE_STATS,     // Cumulative statistics
    MODE_COUNT
};

// ---------- Layout constants (320x240 想定、実寸はM5.Lcd.width()/height()で取得) ---

static constexpr int HEADER_H = 32; // 上部ステータスバー高さ
static constexpr int FOOTER_H = 26; // 下部操作ヒント高さ

// ---------- Globals ---------------------------------------------------------

static BLEManager    bleManager;
static DroneTracker  droneTracker;
static uint8_t       currentWifiChannel = 0; // set to 1 on first advanceSmartScan()
static DisplayMode   displayMode        = MODE_LIST;
static unsigned long lastDisplayMs      = 0;
static volatile bool displayNeedsUpdate = true;

// ---------- Smart scan state ------------------------------------------------

enum ScanPhase { PHASE_SEARCH, PHASE_FOCUS };

// volatile: written by wifiReceiveCallback (WiFi task), read by loop task.
static volatile bool channelHasRid[15] = {}; // index 1-14 valid; 0 unused

static ScanPhase scanPhase       = PHASE_SEARCH;
static bool      focusRefreshing = false;
static int       sweepCount      = 0;
static int       sweepTarget     = 3;

// ---------- WiFi promiscuous callback ---------------------------------------

void wifiReceiveCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) return;

    wifi_promiscuous_pkt_t* ppkt = (wifi_promiscuous_pkt_t*)buf;
    WifiMacHeader*    mac         = (WifiMacHeader*)ppkt->payload;

    uint8_t fc0 = (uint8_t)(mac->frameControl & 0xFF);
    if (fc0 != 0x80 && fc0 != 0x50) return;

    WiFiElementData* elementData = (WiFiElementData*)mac->payload;
    int len = (int)ppkt->rx_ctrl.sig_len - (int)sizeof(WifiMacHeader);
    if (len <= 0) return;

    while (len > 4) {
        if (elementData->id == 221) { // Vendor Specific IE
            vendor_ie_data_t* vi = (vendor_ie_data_t*)elementData;

            if (vi->length >= 4 && isOuiRemoteId(vi)) {
                int payloadLen    = vi->length - 4;
                int notifyDataLen = vi->length + 4;

                uint8_t notifyData[260];
                notifyData[0] = 0x01;
                notifyData[1] = ppkt->rx_ctrl.channel;
                notifyData[2] = (uint8_t)(int8_t)ppkt->rx_ctrl.rssi;
                notifyData[3] = vi->length;
                memcpy(notifyData + 4, vi->vendor_oui, 3);
                notifyData[7] = vi->vendor_oui_type;
                if (payloadLen > 0) {
                    memcpy(notifyData + 8, vi->payload, payloadLen);
                }

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

                uint8_t rxCh = ppkt->rx_ctrl.channel;
                if (rxCh >= WIFI_CHANNEL_MIN && rxCh <= WIFI_CHANNEL_MAX) {
                    channelHasRid[rxCh] = true;
                }
                displayNeedsUpdate = true;
            }
        }

        int dlen = 2 + (int)elementData->length;
        if (dlen <= 0 || dlen > len) break;
        len -= dlen;
        elementData = (WiFiElementData*)((uint8_t*)elementData + dlen);
    }
}

// ---------- Smart scan channel advance --------------------------------------

static uint8_t advanceSmartScan() {
    bool    isSearchMode  = (scanPhase == PHASE_SEARCH || focusRefreshing);
    uint8_t nextCh        = 0;
    bool    sweepComplete = false;

    if (isSearchMode) {
        nextCh = currentWifiChannel + 1;
        if (nextCh > WIFI_CHANNEL_MAX) {
            nextCh        = WIFI_CHANNEL_MIN;
            sweepComplete = true;
        }
    } else {
        for (int i = currentWifiChannel + 1; i <= WIFI_CHANNEL_MAX; i++) {
            if (channelHasRid[i]) { nextCh = (uint8_t)i; break; }
        }
        if (nextCh == 0) {
            sweepComplete = true;
            for (int i = WIFI_CHANNEL_MIN; i <= WIFI_CHANNEL_MAX; i++) {
                if (channelHasRid[i]) { nextCh = (uint8_t)i; break; }
            }
            if (nextCh == 0) nextCh = WIFI_CHANNEL_MIN;
        }
    }

    if (sweepComplete) {
        sweepCount++;
        if (sweepCount >= sweepTarget) {
            sweepCount = 0;
            if (scanPhase == PHASE_SEARCH) {
                bool anyFound = false;
                for (int i = WIFI_CHANNEL_MIN; i <= WIFI_CHANNEL_MAX; i++) {
                    if (channelHasRid[i]) { anyFound = true; break; }
                }
                if (anyFound) {
                    scanPhase   = PHASE_FOCUS;
                    sweepTarget = 120;
                    nextCh      = 0;
                    for (int i = WIFI_CHANNEL_MIN; i <= WIFI_CHANNEL_MAX; i++) {
                        if (channelHasRid[i]) { nextCh = (uint8_t)i; break; }
                    }
                }
            } else if (focusRefreshing) {
                focusRefreshing = false;
                sweepTarget     = 120;
                nextCh          = 0;
                for (int i = WIFI_CHANNEL_MIN; i <= WIFI_CHANNEL_MAX; i++) {
                    if (channelHasRid[i]) { nextCh = (uint8_t)i; break; }
                }
                if (nextCh == 0) nextCh = WIFI_CHANNEL_MIN;
            } else {
                focusRefreshing = true;
                sweepTarget     = 3;
                nextCh          = WIFI_CHANNEL_MIN;
            }
        }
    }

    currentWifiChannel = nextCh;
    return currentWifiChannel;
}

// ---------- Display helpers (320x240, M5GFX経由でCoreS3-SEのタッチパネル機に対応) ---

static void drawHeader() {
    int w = M5.Lcd.width();

    M5.Lcd.fillRect(0, 0, w, HEADER_H, TFT_NAVY);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_NAVY);
    M5.Lcd.setTextSize(2);

    M5.Lcd.setCursor(6, 8);
    M5.Lcd.print("Remote ID");

    // BLE connected indicator
    if (bleManager.isConnected) {
        M5.Lcd.setTextColor(TFT_CYAN, TFT_NAVY);
        M5.Lcd.setCursor(150, 8);
        M5.Lcd.print("BLE");
    }

    // WiFi channel
    M5.Lcd.setTextColor(TFT_WHITE, TFT_NAVY);
    M5.Lcd.setCursor(200, 8);
    M5.Lcd.printf("Ch:%02d", currentWifiChannel);

    // Battery level with colour-coded indicator
    int bat = M5.Power.getBatteryLevel();
    if (bat < 0) bat = 0;
    uint16_t batColor = (bat > 50) ? TFT_GREEN : (bat > 20 ? TFT_YELLOW : TFT_RED);
    M5.Lcd.setTextColor(batColor, TFT_NAVY);
    M5.Lcd.setCursor(w - 54, 8);
    M5.Lcd.printf("%3d%%", bat);
}

static void drawFooterHint(const char* modeLabel) {
    int w = M5.Lcd.width();
    int h = M5.Lcd.height();

    // textSize(2)だと320px幅を超えてはみ出すため、size(1)にして
    // 左詰め「Tap>Mode:...」・右詰め「Hold<Reset」の2要素に分割する。
    // 右詰め位置は textWidth() で実測してから計算するため、
    // フォント幅の変更にも自動追従する。
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    int y = h - FOOTER_H + (FOOTER_H - 8) / 2;

    char left[24];
    snprintf(left, sizeof(left), "Tap>Mode:%s", modeLabel);
    M5.Lcd.setCursor(6, y);
    M5.Lcd.print(left);

    static const char* kRightHint = "Hold<Reset";
    int rightW = M5.Lcd.textWidth(kRightHint);
    M5.Lcd.setCursor(w - rightW - 6, y);
    M5.Lcd.print(kRightHint);
}

static void drawListMode() {
    // RSSI降順で最大3機分のインデックスを取得
    int indices[3];
    int droneCount = droneTracker.getActiveDronesSortedByRssi(indices, 3);
    int totalCount = droneTracker.getActiveDroneCount();

    // ヘッダ部の「機体数」を表示
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.setCursor(6, HEADER_H + 6);
    M5.Lcd.printf("Drones: %d (Top3 RSSI)", totalCount);

    if (droneCount == 0) {
        M5.Lcd.setTextSize(3);
        M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5.Lcd.setCursor(6, HEADER_H + 60);
        M5.Lcd.print("Scanning...");
    } else {
        // 各機体を1行ずつ表示（最大3行）
        // textSize(2)では横320pxに収まらず折り返すため1.5に縮小。
        // 名前欄の最大文字数は textWidth() の実測値から動的に算出する
        // （固定幅フォントのため1文字分の幅で割ればよい）。
        const int nameX = 40;
        M5.Lcd.setTextSize(1.5);
        int charW = M5.Lcd.textWidth("0");
        int maxIdChars = charW > 0 ? (M5.Lcd.width() - nameX - 6) / charW : 18;

        for (int i = 0; i < droneCount; i++) {
            DroneInfo d;
            if (!droneTracker.getDroneAt(indices[i], d)) continue;

            int y = HEADER_H + 34 + i * 40;

            // 機体番号と電波強度（最強機体は黄色強調）
            uint16_t numColor = (i == 0) ? TFT_YELLOW : TFT_WHITE;
            M5.Lcd.setTextColor(numColor, TFT_BLACK);
            M5.Lcd.setCursor(6, y);
            M5.Lcd.printf("#%d", i + 1);

            M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
            M5.Lcd.setCursor(nameX, y);

            // JU登録記号を優先表示、なければシリアル
            // (content areaは毎フレーム全クリアしているためパディング不要)
            if (d.hasRegistrationId) {
                int idLen = strlen(d.registrationId);
                const char* tail = (idLen > maxIdChars) ?
                    (d.registrationId + idLen - maxIdChars) : d.registrationId;
                M5.Lcd.printf("JU:%s", tail);
            } else if (d.hasUasId) {
                int idLen = strlen(d.uasId);
                const char* tail = (idLen > maxIdChars) ?
                    (d.uasId + idLen - maxIdChars) : d.uasId;
                M5.Lcd.printf("SN:%s", tail);
            } else {
                M5.Lcd.print("(waiting...)");
            }

            // 電波強度（次の行）
            uint16_t rssiColor = (d.rssi > -60) ? TFT_GREEN :
                                 (d.rssi > -75) ? TFT_YELLOW : TFT_RED;
            M5.Lcd.setTextColor(rssiColor, TFT_BLACK);
            M5.Lcd.setCursor(nameX, y + 16);
            M5.Lcd.printf("RSSI:%4ddBm Ch:%02d", d.rssi, d.wifiChannel);
        }
    }

    drawFooterHint("LIST");
}

static void drawDetailMode() {
    DroneInfo last;
    bool hasLast = droneTracker.getLastUpdated(last);

    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.setCursor(6, HEADER_H + 6);
    M5.Lcd.print("=== DETAIL ===");

    // textSize(2)だと "UAtype:.. Ch:.. RSSI:..dBm" 等の行が320px幅を
    // 超えて折り返すため1.5に縮小。UAtype行とCh/RSSI行も分割する。
    M5.Lcd.setTextSize(1.5);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

    int lineY = HEADER_H + 34;
    const int lineH = 20;

    if (hasLast) {
        // JU登録記号
        M5.Lcd.setCursor(6, lineY);
        if (last.hasRegistrationId) {
            M5.Lcd.printf("JU:%.24s", last.registrationId);
        } else {
            M5.Lcd.print("JU: (waiting...)");
        }
        lineY += lineH;

        // シリアル番号
        M5.Lcd.setCursor(6, lineY);
        if (last.hasUasId) {
            M5.Lcd.printf("SN:%.24s", last.uasId);
        } else {
            M5.Lcd.print("SN: (waiting...)");
        }
        lineY += lineH;

        M5.Lcd.setCursor(6, lineY);
        M5.Lcd.printf("UAtype:%d", last.uaType);
        lineY += lineH;

        M5.Lcd.setCursor(6, lineY);
        M5.Lcd.printf("Ch:%02d  RSSI:%4ddBm", last.wifiChannel, last.rssi);
        lineY += lineH;

        M5.Lcd.setCursor(6, lineY);
        M5.Lcd.printf("MAC:%02X%02X%02X%02X%02X%02X",
            last.macAddress[0], last.macAddress[1], last.macAddress[2],
            last.macAddress[3], last.macAddress[4], last.macAddress[5]);
        lineY += lineH;

        M5.Lcd.setCursor(6, lineY);
        uint32_t ageSec = (millis() - last.lastSeenMs) / 1000;
        M5.Lcd.printf("Age: %lus ago", ageSec);
    } else {
        M5.Lcd.setCursor(6, lineY);
        M5.Lcd.print("JU: (no data yet)");
        lineY += lineH;
        M5.Lcd.setCursor(6, lineY);
        M5.Lcd.print("SN: (no data yet)");
    }

    drawFooterHint("DETAIL");
}

static void drawStatsMode() {
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.setCursor(6, HEADER_H + 6);
    M5.Lcd.print("=== STATS ===");

    // textSize(2)だと "Name       : ..." 行が320px幅を超えるため1.5に縮小。
    M5.Lcd.setTextSize(1.5);
    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

    int lineY = HEADER_H + 34;
    const int lineH = 20;

    M5.Lcd.setCursor(6, lineY);
    M5.Lcd.printf("Total rcvd : %lu", droneTracker.getTotalReceived());
    lineY += lineH;

    M5.Lcd.setCursor(6, lineY);
    M5.Lcd.printf("Active now : %d", droneTracker.getActiveDroneCount());
    lineY += lineH;

    M5.Lcd.setCursor(6, lineY);
    M5.Lcd.printf("Scan ch    : %02d", currentWifiChannel);
    lineY += lineH;

    M5.Lcd.setCursor(6, lineY);
    M5.Lcd.printf("BLE        : %s", bleManager.isConnected ? "Connected" : "Waiting");
    lineY += lineH;

    M5.Lcd.setCursor(6, lineY);
    M5.Lcd.printf("Name       : %s", bleManager.getDeviceName().c_str());

    drawFooterHint("STATS");
}

static void updateDisplay() {
    drawHeader();

    // Clear content area (below header, down to the bottom of the screen —
    // footer hint is redrawn every cycle as part of each mode's draw function)
    int w = M5.Lcd.width();
    int h = M5.Lcd.height();
    M5.Lcd.fillRect(0, HEADER_H, w, h - HEADER_H, TFT_BLACK);

    switch (displayMode) {
        case MODE_LIST:   drawListMode();   break;
        case MODE_DETAIL: drawDetailMode(); break;
        case MODE_STATS:  drawStatsMode();  break;
        default:          break;
    }
}

// ---------- Touch input -------------------------------------------------------
// 画面のどこをタップしても：表示モード切替（従来のBtnA相当）
// 画面のどこでもホールド（1秒以上、setHoldThreshで設定）：追跡データリセット（従来のBtnB相当）

static void handleTouch() {
    if (M5.Touch.getCount() == 0) return;

    auto t = M5.Touch.getDetail(0);

    if (t.wasClicked()) {
        displayMode = (DisplayMode)((displayMode + 1) % MODE_COUNT);
        displayNeedsUpdate = true;
    }

    if (t.wasHold()) {
        droneTracker.reset();
        displayNeedsUpdate = true;
    }
}

// ---------- setup / loop ----------------------------------------------------

void setup() {
    Serial.begin(115200);

    // Initialize CoreS3-SE (display, touch, power IC) via M5Unified
    auto cfg = M5.config();
    M5.begin(cfg);

    // 誤操作防止のため長押し判定を1秒に設定
    M5.Touch.setHoldThresh(1000);

    // 座標計算どおりに描画するため、意図しない自動改行を無効化
    M5.Lcd.setTextWrap(false, false);

    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setBrightness(80);       // Moderate brightness to save battery

    // Splash screen: FlyDrone logo (320x240)
    M5.Lcd.drawJpg(LOGO_JPEG_CORES3, LOGO_JPEG_CORES3_SIZE, 0, 0);
    delay(2000);
    M5.Lcd.fillScreen(TFT_BLACK);

    // WiFi: station mode + promiscuous receive (management frames only)
    WiFi.mode(WIFI_STA);
    wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifiReceiveCallback);

    // BLE
    uint8_t versionData[] = {1, 0, 0};
    bleManager.startBLE(versionData);

    updateDisplay();
}

void loop() {
    M5.update(); // Refresh touch state

    handleTouch();

    // Advance WiFi channel via smart scan state machine
    uint8_t ch = advanceSmartScan();
    ESP_ERROR_CHECK(esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE));

    // Refresh display at most every DISPLAY_UPDATE_INTERVAL ms
    unsigned long now = millis();
    if (displayNeedsUpdate || (now - lastDisplayMs >= DISPLAY_UPDATE_INTERVAL)) {
        updateDisplay();
        lastDisplayMs      = now;
        displayNeedsUpdate = false;
    }

    delay(500);
}
