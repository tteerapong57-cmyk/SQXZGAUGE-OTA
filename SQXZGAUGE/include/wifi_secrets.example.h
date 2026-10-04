#pragma once
// OPTIONAL fallback network. Normally you set WiFi on the gauge itself:
// P08 -> WIFI button -> pick a network -> type the password on the touch keyboard.
// If you copy this file to include/wifi_secrets.h (git-ignored), it is used only
// until a network has been saved on the gauge.
#define OTA_WIFI_SSID  "YOUR_HOTSPOT_NAME"
#define OTA_WIFI_PASS  "YOUR_HOTSPOT_PASSWORD"
