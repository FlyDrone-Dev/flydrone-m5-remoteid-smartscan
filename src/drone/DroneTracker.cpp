/*
 * SPDX-FileCopyrightText: 2025 Braveridge co., ltd.
 * SPDX-License-Identifier: MIT
 */

#include "DroneTracker.h"
#include <string.h>

// OpenDroneID message type nibbles (upper 4 bits of message byte 0)
static constexpr uint8_t ODID_TYPE_BASIC_ID = 0x0;
static constexpr uint8_t ODID_TYPE_PACK     = 0xF;
static constexpr int     ODID_MSG_SIZE      = 25;

DroneTracker::DroneTracker() : lastUpdatedIndex(-1), totalReceived(0), mutex(nullptr) {
    // Do NOT create mutex here — global ctors may run before FreeRTOS scheduler.
    // Lazy-init on first use (always within task context).
    memset(drones, 0, sizeof(drones));
}

// Internal: create mutex on first call (guaranteed to be in task context)
static inline bool takeMutex(SemaphoreHandle_t& mx, TickType_t ticks) {
    if (mx == nullptr) {
        mx = xSemaphoreCreateMutex();
        if (mx == nullptr) {
            printf("[DroneTracker] mutex create FAILED (heap?)\n");
            return false;
        }
    }
    if (xSemaphoreTake(mx, ticks) != pdTRUE) {
        printf("[DroneTracker] mutex timeout!\n");
        return false;
    }
    return true;
}

// ---------- public ----------------------------------------------------------

void DroneTracker::update(const uint8_t* mac, const uint8_t* payload, int payloadLen,
                          int8_t rssi, uint8_t channel) {
    if (!takeMutex(mutex, pdMS_TO_TICKS(50))) return;

    totalReceived++;
    int idx = findOrCreate(mac);
    if (idx >= 0) {
        DroneInfo& d = drones[idx];
        d.rssi        = rssi;
        d.wifiChannel = channel;
        d.lastSeenMs  = millis();
        d.valid       = true;
        lastUpdatedIndex = idx;
        if (payloadLen > 1) {
            // Diagnostic: print first 4 bytes of payload so we can see message type
            printf("[DT] payload[%d]: %02X %02X %02X %02X ...\n",
                   payloadLen,
                   payload[0],
                   payload[1],
                   payloadLen > 2 ? payload[2] : 0xFF,
                   payloadLen > 3 ? payload[3] : 0xFF);
            // Skip first byte (message counter) - actual OpenDroneID
            // messages start at byte[1] per DJI/ASTM F3411 format
            parseMessage(idx, payload + 1, payloadLen - 1);
            printf("[DT] hasUasId=%d idType=%d uasId=[%s]\n",
                   d.hasUasId, d.idType, d.uasId);
        }
    }

    xSemaphoreGive(mutex);
}

int DroneTracker::getActiveDroneCount() {
    if (!takeMutex(mutex, pdMS_TO_TICKS(20))) return 0;

    uint32_t now = millis();
    int count = 0;
    for (int i = 0; i < MAX_DRONES; i++) {
        if (!drones[i].valid) continue;
        if (now - drones[i].lastSeenMs > DRONE_TIMEOUT_MS) {
            drones[i].valid = false;
        } else {
            count++;
        }
    }

    xSemaphoreGive(mutex);
    return count;
}

bool DroneTracker::getLastUpdated(DroneInfo& out) {
    if (!takeMutex(mutex, pdMS_TO_TICKS(20))) return false;

    bool found = (lastUpdatedIndex >= 0 && drones[lastUpdatedIndex].valid);
    if (found) out = drones[lastUpdatedIndex];

    xSemaphoreGive(mutex);
    return found;
}

void DroneTracker::reset() {
    if (!takeMutex(mutex, pdMS_TO_TICKS(50))) return;
    memset(drones, 0, sizeof(drones));
    lastUpdatedIndex = -1;
    totalReceived    = 0;
    xSemaphoreGive(mutex);
}

// ---------- private ---------------------------------------------------------

int DroneTracker::findOrCreate(const uint8_t* mac) {
    int     oldestIdx  = -1;
    uint32_t oldestMs  = UINT32_MAX;

    for (int i = 0; i < MAX_DRONES; i++) {
        if (!drones[i].valid) {
            // Reuse empty slot
            memcpy(drones[i].macAddress, mac, 6);
            memset(drones[i].uasId, 0, sizeof(drones[i].uasId));
            drones[i].hasUasId = false;
            memset(drones[i].registrationId, 0, sizeof(drones[i].registrationId));
            drones[i].hasRegistrationId = false;
            return i;
        }
        if (memcmp(drones[i].macAddress, mac, 6) == 0) {
            return i;
        }
        if (drones[i].lastSeenMs < oldestMs) {
            oldestMs  = drones[i].lastSeenMs;
            oldestIdx = i;
        }
    }

    // All slots occupied — evict the least recently seen drone
    if (oldestIdx >= 0) {
        memcpy(drones[oldestIdx].macAddress, mac, 6);
        memset(drones[oldestIdx].uasId, 0, sizeof(drones[oldestIdx].uasId));
        drones[oldestIdx].hasUasId = false;
        memset(drones[oldestIdx].registrationId, 0, sizeof(drones[oldestIdx].registrationId));
        drones[oldestIdx].hasRegistrationId = false;
    }
    return oldestIdx;
}

void DroneTracker::parseMessage(int idx, const uint8_t* msg, int len) {
    if (len < 1) return;
    uint8_t msgType = (msg[0] >> 4) & 0x0F;
    printf("  [DT] msgType=0x%X byte0=0x%02X len=%d\n", msgType, msg[0], len);

    if (msgType == ODID_TYPE_BASIC_ID) {
        parseBasicId(idx, msg, len);
    } else if (msgType == ODID_TYPE_PACK) {
        parseMessagePack(idx, msg, len);
    } else {
        printf("  [DT] unknown msgType=0x%X (skipped)\n", msgType);
    }
}

void DroneTracker::parseBasicId(int idx, const uint8_t* msg, int len) {
    // Basic ID message layout:
    //   [0]  = (type_nibble << 4) | version
    //   [1]  = (id_type << 4) | ua_type
    //   [2..21] = UAS ID (20 bytes, null-padded ASCII)
    if (len < 22) return;

    uint8_t idType = (msg[1] >> 4) & 0x0F;
    uint8_t uaType =  msg[1]       & 0x0F;
    drones[idx].idType = idType;
    drones[idx].uaType = uaType;

    // 格納先を idType で分岐
    char* dest;
    int   destSize;
    if (idType == 2) {
        // CAA Registration ID（日本のJU登録記号）
        dest     = drones[idx].registrationId;
        destSize = 20;
    } else {
        // Serial Number（製造シリアル）
        dest     = drones[idx].uasId;
        destSize = 20;
    }

    memcpy(dest, msg + 2, destSize);
    dest[destSize] = '\0';

    // Trim trailing spaces and nulls
    for (int i = destSize - 1; i >= 0; i--) {
        char c = dest[i];
        if (c == '\0' || c == ' ') dest[i] = '\0';
        else break;
    }

    if (idType == 2) {
        drones[idx].hasRegistrationId = true;
    } else {
        drones[idx].hasUasId = true;
    }
}

void DroneTracker::parseMessagePack(int idx, const uint8_t* pack, int len) {
    // Message Pack layout:
    //   [0]  = (0xF << 4) | version
    //   [1]  = single message size (should be 25)
    //   [2]  = number of messages
    //   [3..] = individual messages, each msgSize bytes
    if (len < 3) return;

    uint8_t msgSize  = pack[1];
    uint8_t msgCount = pack[2];

    if (msgSize == 0 || msgSize > ODID_MSG_SIZE) return;

    int offset = 3;
    for (int i = 0; i < msgCount && (offset + msgSize) <= len; i++) {
        parseMessage(idx, pack + offset, msgSize);
        offset += msgSize;
    }
}
