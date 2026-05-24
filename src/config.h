/*
 * SPDX-FileCopyrightText: 2025 Braveridge co., ltd.
 * SPDX-License-Identifier: MIT
 */

#pragma once

// WiFi channel scanning range (2.4GHz band)
#define WIFI_CHANNEL_MIN    1
#define WIFI_CHANNEL_MAX    14

// Maximum SSID length
#define MAX_SSID_LEN        32

// BLE advertising local name
#define ADV_LOCAL_NAME      "BVFDM5C3"

// Firmware version {major, minor, patch}
// NOTE: These UUIDs must match the bv-remote-id original firmware and BvRID app.
//       Verify against the original BLEManager.h before deploying.
#define BLE_SERVICE_UUID        "833bbc00-588e-4ca2-9cd3-717200016c6c"
#define BLE_CHAR_NOTIFY_UUID    "833bbc01-588e-4ca2-9cd3-717200016c6c"
#define BLE_CHAR_VERSION_UUID   "833bbc02-588e-4ca2-9cd3-717200016c6c"

// Display refresh interval (ms)
#define DISPLAY_UPDATE_INTERVAL 500

// Maximum number of simultaneously tracked drones
#define MAX_DRONES              20

// Drone entry expires after this many ms without a new beacon
#define DRONE_TIMEOUT_MS        30000
