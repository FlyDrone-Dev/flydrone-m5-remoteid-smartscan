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

// ---------- Screen sleep constants ------------------------------------------

static constexpr uint8_t       NORMAL_BRIGHTNESS = 80;    // 通常時の輝度
static constexpr unsigned long SCREEN_SLEEP_MS   = 30000; // 無操作でバックライト消灯するまでの時間

// ---------- Globals ---------------------------------------------------------

static BLEManager    bleManager;
static DroneTracker  droneTracker;
static uint8_t       currentWifiChannel  = 0; // set to 1 on first advanceSmartScan()
static DisplayMode   displayMode         = MODE_LIST;
static unsigned long lastDisplayMs       = 0;
static volatile bool displayNeedsUpdate  = true;
static bool          displaySleeping     = false;
static unsigned long lastActivityMs      = 0;

// ---------- Smart scan state ------------------------------------------------

// PHASE_SEARCH1: 1-14ch を3スイープして受信CHを探す（起動時 / 記憶CH消失時）
// PHASE_FOCUS  : 記憶CHのみを120スイープする濃密フェーズ
// PHASE_SEARCH2: 各新規探索CHの直後に「記憶CH×2」を挟むインターリーブ探索（3スイープ）
enum ScanPhase { PHASE_SEARCH1, PHASE_FOCUS, PHASE_SEARCH2 };

// volatile: written by wifiReceiveCallback (WiFi task), read by loop task.
// Single-byte write on ESP32 is atomic; no mutex needed, but volatile prevents
// the compiler from caching the value in a register across task boundaries.
static volatile bool channelHasRid[15] = {}; // index 1-14 valid; 0 unused
static volatile bool lastSeenFlag[15]  = {}; // 判定期間中に受信があったか（コールバックがtrueにする）
static int           missCount[15]     = {}; // 連続で受信なしと判定された回数（0リセット、3で解放）

static volatile ScanPhase scanPhase = PHASE_SEARCH1;
static int       sweepCount  = 0;
static int       sweepTarget = 3;

// PHASE_SEARCH2 専用の内部状態
static int            search2Index      = 0;              // 現スイープ内のスロット位置 0..(14*3-1)
static int            memRotateIdx      = WIFI_CHANNEL_MIN;// 記憶CHローテーションの現在位置
static volatile bool  search2SawActivity = false;         // SEARCH2の3スイープ中に受信があったか

// ---------- WiFi promiscuous callback ---------------------------------------

// Note: IRAM_ATTR removed — promiscuous cb runs in WiFi task, not ISR.
// IRAM_ATTR would prevent calling flash-resident DroneTracker/BLE functions safely.
void wifiReceiveCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
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

                // Record this channel as having received RID for smart scan
                uint8_t rxCh = ppkt->rx_ctrl.channel;
                if (rxCh >= WIFI_CHANNEL_MIN && rxCh <= WIFI_CHANNEL_MAX) {
                    channelHasRid[rxCh] = true;
                    lastSeenFlag[rxCh]  = true; // 判定期間中の受信を記録（FOCUS/SEARCH2問わず）
                    if (scanPhase == PHASE_SEARCH2) {
                        // SEARCH2中の受信 = 新規CH発見 or 記憶CH受信（いずれも「活動あり」）
                        search2SawActivity = true;
                    }
                }
                displayNeedsUpdate = true;
            }
        }

        int dlen = 2 + (int)elementData->length;
        if (dlen <= 0 || dlen > len) break; // Malformed IE: stop
        len -= dlen;
        elementData = (WiFiElementData*)((uint8_t*)elementData + dlen);
    }
}

// ---------- Smart scan channel advance --------------------------------------

// 記憶CH群（channelHasRid=true）からローテーションで次の1つを返す。無ければ0。
// 記憶CH数によらず SEARCH2 の「記憶CH×2」スロットを公平に埋めるための連続ローテーション。
static uint8_t nextMemoryChannel() {
    for (int step = 0; step < (WIFI_CHANNEL_MAX - WIFI_CHANNEL_MIN + 1); step++) {
        memRotateIdx++;
        if (memRotateIdx > WIFI_CHANNEL_MAX) memRotateIdx = WIFI_CHANNEL_MIN;
        if (channelHasRid[memRotateIdx]) return (uint8_t)memRotateIdx;
    }
    return 0; // 記憶CHなし
}

static uint8_t advanceSmartScan() {
    uint8_t nextCh        = 0;
    bool    sweepComplete = false;

    // ---- フェーズ別の次CH決定 --------------------------------------------
    if (scanPhase == PHASE_SEARCH1) {
        // 1-14ch を順に走査（1周で1スイープ）
        nextCh = currentWifiChannel + 1;
        if (nextCh > WIFI_CHANNEL_MAX) {
            nextCh        = WIFI_CHANNEL_MIN;
            sweepComplete = true;
        }
    } else if (scanPhase == PHASE_FOCUS) {
        // 記憶CHのみを順に走査
        for (int i = currentWifiChannel + 1; i <= WIFI_CHANNEL_MAX; i++) {
            if (channelHasRid[i]) { nextCh = (uint8_t)i; break; }
        }
        if (nextCh == 0) {
            sweepComplete = true;
            for (int i = WIFI_CHANNEL_MIN; i <= WIFI_CHANNEL_MAX; i++) {
                if (channelHasRid[i]) { nextCh = (uint8_t)i; break; }
            }
            if (nextCh == 0) nextCh = WIFI_CHANNEL_MIN; // safety: 記憶CH消失時
        }
    } else { // PHASE_SEARCH2
        // 各新規探索CHの直後に「記憶CH×2」を挟むインターリーブ走査。
        // 1スイープ = CH1,mem,mem, CH2,mem,mem, ... CH14,mem,mem （14*3スロット）。
        int slot = search2Index % 3;
        if (slot == 0) {
            nextCh = (uint8_t)(search2Index / 3 + WIFI_CHANNEL_MIN); // 新規探索CH 1..14
        } else {
            nextCh = nextMemoryChannel();                            // 記憶CH（ローテーション）
            if (nextCh == 0) {
                // 記憶CHが無い場合は新規探索CHで埋める（安全側）
                nextCh = (uint8_t)(search2Index / 3 + WIFI_CHANNEL_MIN);
            }
        }
        search2Index++;
        if (search2Index >= (WIFI_CHANNEL_MAX - WIFI_CHANNEL_MIN + 1) * 3) {
            search2Index  = 0;
            sweepComplete = true;
        }
    }

    // ---- スイープ完了時のフェーズ遷移処理 --------------------------------
    if (sweepComplete) {
        sweepCount++;
        if (sweepCount >= sweepTarget) {
            sweepCount = 0;

            if (scanPhase == PHASE_SEARCH1) {
                // 3スイープ完了：受信CHがあれば FOCUS へ、無ければ更に3スイープ継続
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
                // else: SEARCH1 継続（sweepTargetは3のまま、次の3スイープへ）

            } else if (scanPhase == PHASE_FOCUS) {
                // 120スイープ完了：SEARCH2 へ移行
                scanPhase          = PHASE_SEARCH2;
                sweepTarget        = 3;
                search2SawActivity = false;
                nextCh             = WIFI_CHANNEL_MIN; // 先頭スロット(CH1)を出す
                search2Index       = 1;                // 次呼び出しは slot1(記憶CH) から

            } else { // PHASE_SEARCH2 完了（3スイープ）
                // --- 記憶CH解放判定（3サイクルmissカウント方式）---
                for (int i = WIFI_CHANNEL_MIN; i <= WIFI_CHANNEL_MAX; i++) {
                    if (!channelHasRid[i]) continue;
                    if (lastSeenFlag[i]) {
                        // 判定期間中に受信あり → missリセット、フラグを戻す
                        missCount[i]    = 0;
                        lastSeenFlag[i] = false;
                    } else {
                        // 判定期間中に一度も受信なし → miss加算
                        missCount[i]++;
                        if (missCount[i] >= 3) {
                            // 3サイクル連続miss → 記憶から解放（スロットを空ける）
                            channelHasRid[i] = false;
                            missCount[i]     = 0;
                            lastSeenFlag[i]  = false;
                        }
                    }
                }

                // --- 残存記憶CH数 ---
                int memCount = 0;
                for (int i = WIFI_CHANNEL_MIN; i <= WIFI_CHANNEL_MAX; i++) {
                    if (channelHasRid[i]) memCount++;
                }

                // --- 次フェーズ決定 ---
                if (memCount == 0) {
                    // 記憶CHが全て解放され0個 → 起動時と同じ SEARCH1 へ明示遷移
                    scanPhase   = PHASE_SEARCH1;
                    sweepTarget = 3;
                    nextCh      = WIFI_CHANNEL_MIN;
                } else if (!search2SawActivity) {
                    // 3スイープ中に新規CH発見も記憶CH受信も無し → SEARCH1（記憶CH保持）
                    scanPhase   = PHASE_SEARCH1;
                    sweepTarget = 3;
                    nextCh      = WIFI_CHANNEL_MIN;
                } else {
                    // それ以外 → FOCUS に戻る
                    scanPhase   = PHASE_FOCUS;
                    sweepTarget = 120;
                    nextCh      = 0;
                    for (int i = WIFI_CHANNEL_MIN; i <= WIFI_CHANNEL_MAX; i++) {
                        if (channelHasRid[i]) { nextCh = (uint8_t)i; break; }
                    }
                    if (nextCh == 0) nextCh = WIFI_CHANNEL_MIN;
                }
            }
        }
    }

    currentWifiChannel = nextCh;

    // ---- デバッグ出力（フェーズ名・周回数・記憶CH一覧・各missCount）----
    const char* phaseName = (scanPhase == PHASE_SEARCH1) ? "SEARCH1" :
                            (scanPhase == PHASE_FOCUS)   ? "FOCUS"   : "SEARCH2";
    char memList[96];
    int  pos = 0;
    memList[0] = '\0';
    for (int i = WIFI_CHANNEL_MIN;
         i <= WIFI_CHANNEL_MAX && pos < (int)sizeof(memList) - 12; i++) {
        if (channelHasRid[i]) {
            pos += snprintf(memList + pos, sizeof(memList) - pos,
                            "%d(miss%d) ", i, missCount[i]);
        }
    }
    if (pos == 0) snprintf(memList, sizeof(memList), "(none)");
    printf("[%s] sweep:%d/%d ch:%d mem:%s\n",
           phaseName, sweepCount, sweepTarget, currentWifiChannel, memList);

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
    // RSSI降順で最大3機分のインデックスを取得
    int indices[3];
    int droneCount = droneTracker.getActiveDronesSortedByRssi(indices, 3);
    int totalCount = droneTracker.getActiveDroneCount();

    // ヘッダ部の「機体数」を表示
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.setCursor(4, 22);
    M5.Lcd.printf("Drones: %d (Top3 by RSSI)", totalCount);

    if (droneCount == 0) {
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5.Lcd.setCursor(4, 55);
        M5.Lcd.print("Scanning...");
    } else {
        // 各機体を1行ずつ表示（最大3行）
        M5.Lcd.setTextSize(1);
        for (int i = 0; i < droneCount; i++) {
            DroneInfo d;
            if (!droneTracker.getDroneAt(indices[i], d)) continue;

            int y = 36 + i * 28;

            // 機体番号と電波強度（最強機体は黄色強調）
            uint16_t numColor = (i == 0) ? TFT_YELLOW : TFT_WHITE;
            M5.Lcd.setTextColor(numColor, TFT_BLACK);
            M5.Lcd.setCursor(4, y);
            M5.Lcd.printf("#%d", i + 1);

            M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
            M5.Lcd.setCursor(24, y);

            // JU登録記号を優先表示、なければシリアル
            if (d.hasRegistrationId) {
                int idLen = strlen(d.registrationId);
                const char* tail = (idLen > 14) ?
                    (d.registrationId + idLen - 14) : d.registrationId;
                M5.Lcd.printf("JU:%-14s", tail);
            } else if (d.hasUasId) {
                int idLen = strlen(d.uasId);
                const char* tail = (idLen > 14) ?
                    (d.uasId + idLen - 14) : d.uasId;
                M5.Lcd.printf("SN:%-14s", tail);
            } else {
                M5.Lcd.print("(waiting...)  ");
            }

            // 電波強度（次の行）
            uint16_t rssiColor = (d.rssi > -60) ? TFT_GREEN :
                                 (d.rssi > -75) ? TFT_YELLOW : TFT_RED;
            M5.Lcd.setTextColor(rssiColor, TFT_BLACK);
            M5.Lcd.setCursor(24, y + 12);
            M5.Lcd.printf("RSSI:%4ddBm Ch:%02d", d.rssi, d.wifiChannel);
        }
    }

    // ボタンヒント
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

    if (hasLast) {
        // JU登録記号
        M5.Lcd.setCursor(4, 36);
        if (last.hasRegistrationId) {
            M5.Lcd.printf("JU:%.20s", last.registrationId);
        } else {
            M5.Lcd.print("JU: (waiting...)    ");
        }
        // シリアル番号
        M5.Lcd.setCursor(4, 50);
        if (last.hasUasId) {
            M5.Lcd.printf("SN:%.20s", last.uasId);
        } else {
            M5.Lcd.print("SN: (waiting...)    ");
        }
        M5.Lcd.setCursor(4, 64);
        M5.Lcd.printf("UAtype:%d", last.uaType);
    } else {
        M5.Lcd.setCursor(4, 36);
        M5.Lcd.print("JU: (no data yet)   ");
        M5.Lcd.setCursor(4, 50);
        M5.Lcd.print("SN: (no data yet)   ");
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

    M5.Lcd.setCursor(4, 92);
    M5.Lcd.printf("Name       : %s", bleManager.getDeviceName().c_str());

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

// ---------- Screen sleep ------------------------------------------------------
// 30秒間無操作でバックライトを消灯する。Wi-Fi受信・スマートスキャン・BLE転送・
// DroneTrackerの更新はスリープ中も継続する（このsleep機構は描画の輝度制御のみ）。

static void sleepDisplay() {
    if (displaySleeping) return;
    displaySleeping = true;
    M5.Lcd.setBrightness(0);
}

static void wakeDisplay() {
    displaySleeping = false;
    M5.Lcd.setBrightness(NORMAL_BRIGHTNESS);
    lastActivityMs = millis();
}

// ---------- Button input ------------------------------------------------------
// Aボタン：表示モード切替 / Bボタン：追跡データリセット。
// 消灯中の最初の押下（A/Bどちらでも）はバックライト復帰のみに使い、
// モード切替/リセットとしては処理しない。2回目以降の押下から通常処理を再開する。

static void handleButtons() {
    bool pressedA = M5.BtnA.wasPressed();
    bool pressedB = M5.BtnB.wasPressed();

    if (!pressedA && !pressedB) return;

    // 消灯中の1回目の押下：復帰のみ（モード切替/リセットは行わない）
    if (displaySleeping) {
        wakeDisplay();
        return;
    }

    lastActivityMs = millis();

    // Button A: cycle through display modes
    if (pressedA) {
        displayMode = (DisplayMode)((displayMode + 1) % MODE_COUNT);
        displayNeedsUpdate = true;
    }

    // Button B: clear all tracked drone data
    if (pressedB) {
        droneTracker.reset();
        displayNeedsUpdate = true;
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
    M5.Lcd.setBrightness(NORMAL_BRIGHTNESS); // Moderate brightness to save battery

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
    bleManager.startBLE(versionData);

    lastActivityMs = millis();
    updateDisplay();
}

void loop() {
    M5.update(); // Refresh button state

    handleButtons();

    // Advance WiFi channel via smart scan state machine
    // (Wi-Fi受信・スマートスキャン・BLE転送・DroneTrackerの更新はスリープ中も継続)
    uint8_t ch = advanceSmartScan();
    ESP_ERROR_CHECK(esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE));

    unsigned long now = millis();

    // 30秒間無操作ならバックライトを消灯
    if (!displaySleeping && (now - lastActivityMs >= SCREEN_SLEEP_MS)) {
        sleepDisplay();
    }

    // Refresh display at most every DISPLAY_UPDATE_INTERVAL ms
    if (displayNeedsUpdate || (now - lastDisplayMs >= DISPLAY_UPDATE_INTERVAL)) {
        updateDisplay();
        lastDisplayMs      = now;
        displayNeedsUpdate = false;
    }

    delay(500);
}
