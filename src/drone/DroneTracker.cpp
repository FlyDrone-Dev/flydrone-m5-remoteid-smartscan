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

DroneTracker::DroneTracker() : lastUpdatedIndex(-1), totalReceived(0) {
    mutex = xSemaphoreCreateMutex();
    memset(drones, 0, sizeof(drones));
}

// ---------- public ----------------------------------------------------------

void DroneTracker::update(const uint8_t* mac, const uint8_t* payload, int payloadLen,
                          int8_t rssi, uint8_t channel) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    totalReceived++;
    int idx = findOrCreate(mac);
    if (idx >= 0) {
        DroneInfo& d = drones[idx];
        d.rssi        = rssi;
        d.wifiChannel = channel;
        d.lastSeenMs  = millis();
        d.valid       = true;
        lastUpdatedIndex = idx;
        if (payloadLen > 0) {
            parseMessage(idx, payload, payloadLen);
        }
    }

    xSemaphoreGive(mutex);
}

int DroneTracker::getActiveDroneCount() {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) return 0;

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
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;

    bool found = (lastUpdatedIndex >= 0 && drones[lastUpdatedIndex].valid);
    if (found) out = drones[lastUpdatedIndex];

    xSemaphoreGive(mutex);
    return found;
}

void DroneTracker::reset() {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
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
    }
    return oldestIdx;
}

void DroneTracker::parseMessage(int idx, const uint8_t* msg, int len) {
    if (len < 1) return;
    uint8_t msgType = (msg[0] >> 4) & 0x0F;

    if (msgType == ODID_TYPE_BASIC_ID) {
        parseBasicId(idx, msg, len);
    } else if (msgType == ODID_TYPE_PACK) {
        parseMessagePack(idx, msg, len);
    }
}

void DroneTracker::parseBasicId(int idx, const uint8_t* msg, int len) {
    // Basic ID message layout:
    //   [0]  = (type_nibble << 4) | version
    //   [1]  = (id_type << 4) | ua_type
    //   [2..21] = UAS ID (20 bytes, null-padded ASCII)
    if (len < 22) return;

    drones[idx].idType = (msg[1] >> 4) & 0x0F;
    drones[idx].uaType =  msg[1]       & 0x0F;

    memcpy(drones[idx].uasId, msg + 2, 20);
    drones[idx].uasId[20] = '\0';

    // Trim trailing spaces and nulls
    for (int i = 19; i >= 0; i--) {
        char c = drones[idx].uasId[i];
        if (c == '\0' || c == ' ') {
            drones[idx].uasId[i] = '\0';
        } else {
            break;
        }
    }
    drones[idx].hasUasId = true;
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
