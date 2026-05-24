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

class BLEManager : public BLEServerCallbacks {
public:
    bool isConnected = false;

    void startBLE(const char* localName, uint8_t* versionData);
    void notifyRidData(uint8_t* data, int length);

private:
    BLEServer*         pServer      = nullptr;
    BLECharacteristic* pNotifyChar  = nullptr;

    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;
};
