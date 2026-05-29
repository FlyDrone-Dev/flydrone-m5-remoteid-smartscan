/*
 * SPDX-FileCopyrightText: 2025 Braveridge co., ltd.
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "../config.h"

struct DroneInfo {
    uint8_t  macAddress[6];
    char     uasId[21];     // ANSI/CTA-2063-A serial (20 chars + NUL)
    int8_t   rssi;
    uint8_t  wifiChannel;
    uint32_t lastSeenMs;
    uint8_t  idType;        // OpenDroneID ID type (1=Serial, 2=CAA, ...)
    uint8_t  uaType;        // UA type
    bool     hasUasId;
    char     registrationId[21]; // JU登録記号 idType=4
    bool     hasRegistrationId;
    bool     valid;
};

class DroneTracker {
public:
    DroneTracker();

    // Called from WiFi promiscuous callback task — mutex protected
    void update(const uint8_t* mac, const uint8_t* payload, int payloadLen,
                int8_t rssi, uint8_t channel);

    int  getActiveDroneCount();
    bool getLastUpdated(DroneInfo& out);
    void reset();

    // RSSI降順でソートされた有効ドローンインデックスを取得
    // 戻り値: 取得した機体数
    int  getActiveDronesSortedByRssi(int* outIndices, int maxCount);

    // 指定インデックスのDroneInfoを取得
    bool getDroneAt(int index, DroneInfo& out);

    uint32_t getTotalReceived() const { return totalReceived; }

private:
    DroneInfo         drones[MAX_DRONES];
    int               lastUpdatedIndex;
    volatile uint32_t totalReceived;
    SemaphoreHandle_t mutex;

    int  findOrCreate(const uint8_t* mac);
    void parseMessage(int idx, const uint8_t* msg, int len);
    void parseBasicId(int idx, const uint8_t* msg, int len);
    void parseMessagePack(int idx, const uint8_t* pack, int len);
};
