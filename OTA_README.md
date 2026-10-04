# SQXZGAUGE – OTA update via GitHub Releases

## How it works
1. You push a tag (e.g. `v7.11`). GitHub Actions builds the firmware and creates a Release
   with two files: `firmware.bin` and `version.txt` (`<version> <md5>`).
2. On the gauge: open **P08 (SYSTEM HEALTH)** and press **UPDATE** (tap twice, only works when stopped),
   or **hold a finger on the screen while powering on**. It connects to your
   phone hotspot, reads `version.txt`, and if the version differs it downloads
   `firmware.bin`, checks the MD5, flashes, and reboots. No finger = normal boot (no WiFi at all
   while riding).

## P08 buttons
- **WIFI** – shows the saved network ("SET WIFI" if none). Tap twice (and only when stopped) to reboot into
  the on-device WiFi setup: scan -> pick a network -> type the password on the touch keyboard -> connection
  test -> saved to NVS (survives OTA updates). CANCEL/idle 2 min = normal boot.
- **UPDATE** – 1st tap arms ("TAP AGAIN", 3 s), 2nd tap reboots straight into OTA mode.
  Refused while GPS speed > 3 km/h ("STOP FIRST") or if no WiFi is saved.

## One-time setup
1. (Optional) `include/wifi_secrets.example.h` -> `include/wifi_secrets.h` as a fallback network. Normally
   skip this and use P08 -> WIFI instead. `wifi_secrets.h` is git-ignored (the repo is public).
2. Flash once over USB (`pio run -t upload`). The partition table changed (two OTA slots),
   so this first flash must be USB. `LittleFS` is re-formatted (old run logs are cleared).
3. Put the project at the **root** of the GitHub repo (`platformio.ini` at top level), commit, push.

## Releasing an update
```
git tag v7.11
git push origin v7.11
```
Wait for the Action to finish (Actions tab), then do the touch-hold boot on the gauge.

## Rollback
`version.txt` is compared with `!=`, so releasing an older version number
(e.g. tag `v7.10` again as a new release) downgrades.
If a new build crashes after the OTA check in `setup()`, OTA mode still works (hold touch at boot).
If it crashes *before* that point, re-flash by USB.
