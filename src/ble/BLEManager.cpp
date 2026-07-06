/*
 * SPDX-FileCopyrightText: 2025 Braveridge co., ltd.
 * SPDX-License-Identifier: MIT
 */

#include "BLEManager.h"
#include "../config.h"

#include <Preferences.h>
#include <WiFi.h>
#include <cctype>

String BLEManager::resolveDeviceName() {
    Preferences prefs;
    prefs.begin(BLE_NVS_NAMESPACE, /*readOnly=*/true);
    String customName = prefs.getString(BLE_NVS_KEY_DEVNAME, "");
    prefs.end();

    if (customName.length() > 0) {
        return customName;
    }

    uint8_t mac[6];
    WiFi.macAddress(mac);
    char buf[24];
    snprintf(buf, sizeof(buf), "%s-%02X%02X", BLE_NAME_PREFIX, mac[4], mac[5]);
    return String(buf);
}

void BLEManager::startBLE(uint8_t* versionData) {
    currentDeviceName = resolveDeviceName();
    BLEDevice::init(currentDeviceName.c_str());

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(this);

    BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

    // Notify characteristic: used to forward Remote ID beacon data to the app
    pNotifyChar = pService->createCharacteristic(
        BLE_CHAR_NOTIFY_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pNotifyChar->addDescriptor(new BLE2902());

    // Version characteristic: readable firmware version {major, minor, patch}
    BLECharacteristic* pVersionChar = pService->createCharacteristic(
        BLE_CHAR_VERSION_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    pVersionChar->setValue(versionData, 3);

    // Config characteristic (FD-DEV-2607-031): write-only rename target.
    // Persists to NVS; new name takes effect on next boot.
    BLECharacteristic* pConfigChar = pService->createCharacteristic(
        BLE_CHAR_CONFIG_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pConfigChar->setCallbacks(this);

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
}

void BLEManager::notifyRidData(uint8_t* data, int length) {
    if (!isConnected || pNotifyChar == nullptr) return;
    pNotifyChar->setValue(data, length);
    pNotifyChar->notify();
}

void BLEManager::onConnect(BLEServer* /*pServer*/) {
    isConnected = true;
}

void BLEManager::onDisconnect(BLEServer* /*pServer*/) {
    isConnected = false;
    // Restart advertising so the app can reconnect
    BLEDevice::startAdvertising();
}

void BLEManager::onWrite(BLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value.length() > BLE_NAME_MAX_LEN) return; // reject: leave NVS unchanged

    for (char c : value) {
        if (!(std::isalnum((unsigned char)c) || c == '-')) return; // reject: invalid char
    }

    Preferences prefs;
    prefs.begin(BLE_NVS_NAMESPACE, /*readOnly=*/false);
    if (value.empty()) {
        prefs.remove(BLE_NVS_KEY_DEVNAME); // empty write -> revert to MAC-derived default
    } else {
        prefs.putString(BLE_NVS_KEY_DEVNAME, value.c_str());
    }
    prefs.end();
    // Applied on next boot only — no live BLEDevice::init()/advertising restart here.
}
