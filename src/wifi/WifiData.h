/*
 * SPDX-FileCopyrightText: 2025 Braveridge co., ltd.
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <esp_wifi_types.h>

// IEEE 802.11 beacon frame fixed fields (packed, byte-accurate layout)
#pragma pack(push, 1)
struct WifiMacHeader {
    uint16_t frameControl;
    uint16_t duration;
    uint8_t  destinationAddress[6];
    uint8_t  sourceAddress[6];
    uint8_t  bssid[6];
    uint16_t sequenceControl;   // [3:0]=fragment, [15:4]=sequence
    uint8_t  timestamp[8];      // Beacon fixed field
    uint16_t beaconInterval;    // Beacon fixed field
    uint16_t capabilityInfo;    // Beacon fixed field
    uint8_t  payload[0];        // Information Elements start here
};
#pragma pack(pop)

// IEEE 802.11 Information Element (tagged parameter)
#pragma pack(push, 1)
struct WiFiElementData {
    uint8_t id;
    uint8_t length;
    uint8_t payload[0];
};
#pragma pack(pop)

// Returns true if the vendor IE belongs to FAA/ASTM F3411 Remote ID.
// OUI: FA:0B:BC  Type: 0x0D
inline bool isOuiRemoteId(vendor_ie_data_t* vi) {
    if (vi == nullptr || vi->length < 4) return false;
    return (vi->vendor_oui[0] == 0xFA &&
            vi->vendor_oui[1] == 0x0B &&
            vi->vendor_oui[2] == 0xBC &&
            vi->vendor_oui_type == 0x0D);
}
