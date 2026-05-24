/*
 * SPDX-FileCopyrightText: 2025 Braveridge co., ltd.
 * SPDX-License-Identifier: MIT
 */

#include "BLEManager.h"
#include "../config.h"

void BLEManager::startBLE(const char* localName, uint8_t* versionData) {
    BLEDevice::init(localName);

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
