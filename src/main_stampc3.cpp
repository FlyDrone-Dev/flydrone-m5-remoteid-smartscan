/*
 * SPDX-FileCopyrightText: 2025 Braveridge co., ltd.
 * SPDX-License-Identifier: MIT
 *
 * Remote ID receiver firmware for M5Stamp C3 (ESP32-C3).
 * Ported from the M5StickC Plus2 build (main.cpp).
 *
 * M5Stamp C3 は LCD・物理2ボタン・バッテリIC を搭載しないため、
 * 表示は内蔵RGB LED(SK6812, GPIO2)の3色ステータス表示に、
 * 操作は内蔵ユーザーボタン(GPIO3)の長押しによるデータリセットに置き換える。
 *
 * スマートスキャン状態機・BLE転送・DroneTracker連携・Wi-Fiプロミスキャス受信の
 * ロジックは main.cpp と同一（コピー元）。実機未検証（コンパイル確認のみ）。
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"
#include "ble/BLEManager.h"
#include "wifi/WifiData.h"
#include "drone/DroneTracker.h"

// ---------- Hardware pins (M5Stamp C3) --------------------------------------

static constexpr uint8_t  PIN_RGB_LED   = 2;  // 内蔵SK6812 RGB LED
static constexpr uint8_t  PIN_USER_BTN  = 3;  // 内蔵ユーザーボタン（GNDプル、押下でLOW）
static constexpr uint8_t  LED_BRIGHTNESS = 32; // 0-255（眩しさ抑制のため控えめに）
static constexpr unsigned long BTN_HOLD_MS = 1000; // 長押し判定（1秒）でリセット

// ---------- Globals ---------------------------------------------------------

static BLEManager    bleManager;
static DroneTracker  droneTracker;
static uint8_t       currentWifiChannel = 0; // set to 1 on first advanceSmartScan()

// 内蔵RGB LED（1灯）
static Adafruit_NeoPixel led(1, PIN_RGB_LED, NEO_GRB + NEO_KHZ800);

// ボタン長押し検知の内部状態
static bool          btnWasDown   = false; // 直前ループで押下中だったか
static unsigned long btnDownMs    = 0;     // 押下開始時刻
static bool          btnHoldFired = false; // 今回の押下で既にリセット発火済みか

// ---------- Smart scan state ------------------------------------------------
// （main.cpp と同一：PHASE_SEARCH1/FOCUS/SEARCH2 と記憶CH解放ロジック）

// PHASE_SEARCH1: 1-14ch を3スイープして受信CHを探す（起動時 / 記憶CH消失時）
// PHASE_FOCUS  : 記憶CHのみを120スイープする濃密フェーズ
// PHASE_SEARCH2: 各新規探索CHの直後に「記憶CH×2」を挟むインターリーブ探索（3スイープ）
enum ScanPhase { PHASE_SEARCH1, PHASE_FOCUS, PHASE_SEARCH2 };

// volatile: written by wifiReceiveCallback (WiFi task), read by loop task.
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

// ---------- RGB LED status --------------------------------------------------
// 3状態表示（独自タイマーは持たず、getActiveDroneCount()の自然な遷移を利用）:
//   赤 : BLE未リンク（起動直後〜接続待ち）
//   青 : BLEリンク済み・リモートID未受信（getActiveDroneCount()==0）
//   緑 : BLEリンク済み・リモートID受信中（getActiveDroneCount()>0）

static void updateStatusLed() {
    uint8_t r = 0, g = 0, b = 0;

    if (!bleManager.isConnected) {
        r = LED_BRIGHTNESS;                       // 赤
    } else if (droneTracker.getActiveDroneCount() == 0) {
        b = LED_BRIGHTNESS;                       // 青
    } else {
        g = LED_BRIGHTNESS;                       // 緑
    }

    led.setPixelColor(0, led.Color(r, g, b));
    led.show();
}

// ---------- User button (GPIO3) ---------------------------------------------
// 長押し（BTN_HOLD_MS 以上）でトラッキングデータをリセットする。
// 押下でLOW（内蔵プルアップ利用）。loop()のdelay(500)より細かい判定は不要なため、
// millis()のタイムスタンプ差で長押し時間を計測する（ループ周期に依存しない）。

static void handleUserButton() {
    bool down = (digitalRead(PIN_USER_BTN) == LOW);
    unsigned long now = millis();

    if (down && !btnWasDown) {
        // 押下開始
        btnDownMs    = now;
        btnHoldFired = false;
    } else if (down && btnWasDown && !btnHoldFired) {
        // 押下継続中：長押し閾値に達したらリセット発火（1回のみ）
        if (now - btnDownMs >= BTN_HOLD_MS) {
            droneTracker.reset();
            btnHoldFired = true;
            printf("[BTN] long-press → droneTracker.reset()\n");
        }
    }
    // 離した場合は次回押下開始で状態がリセットされる

    btnWasDown = down;
}

// ---------- setup / loop ----------------------------------------------------

void setup() {
    Serial.begin(115200);

    // 内蔵ユーザーボタン（押下でLOW）
    pinMode(PIN_USER_BTN, INPUT_PULLUP);

    // 内蔵RGB LED初期化：起動直後は赤（BLE未リンク）
    led.begin();
    led.setBrightness(255); // 明るさは色値側(LED_BRIGHTNESS)で制御するため最大に
    updateStatusLed();

    // WiFi: station mode + promiscuous receive (management frames only)
    WiFi.mode(WIFI_STA);
    wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifiReceiveCallback);

    // BLE
    uint8_t versionData[] = {1, 0, 0};
    bleManager.startBLE(versionData);

    updateStatusLed();
}

void loop() {
    // 内蔵ユーザーボタン（長押しでリセット）
    handleUserButton();

    // Advance WiFi channel via smart scan state machine
    uint8_t ch = advanceSmartScan();
    ESP_ERROR_CHECK(esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE));

    // ステータスLEDを更新（BLEリンク状態・受信有無を反映）
    updateStatusLed();

    delay(500);
}
