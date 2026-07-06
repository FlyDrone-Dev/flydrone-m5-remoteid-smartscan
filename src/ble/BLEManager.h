/*
 * SPDX-FileCopyrightText: 2025 Braveridge co., ltd.
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>

// BLEServerCallbacks: connection tracking.
// BLECharacteristicCallbacks: handles writes to the config (rename) characteristic.
class BLEManager : public BLEServerCallbacks, public BLECharacteristicCallbacks {
public:
    bool isConnected = false;

    void startBLE(uint8_t* versionData);
    void notifyRidData(uint8_t* data, int length);
    String getDeviceName() const { return currentDeviceName; }

private:
    BLEServer*         pServer      = nullptr;
    BLECharacteristic* pNotifyChar  = nullptr;
    String             currentDeviceName;

    // FD-DEV-2607-031: resolves NVS-stored custom name, or falls back to
    // "<PREFIX>-XXXX" generated from the factory MAC address.
    String resolveDeviceName();

    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;

    // Config characteristic write: validates and persists a new device name
    // to NVS. Reflected on next boot only — does not restart advertising.
    void onWrite(BLECharacteristic* pCharacteristic) override;
};
