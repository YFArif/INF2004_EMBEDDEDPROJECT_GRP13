# Portable mosquitto for Windows

mosquitto 2.1.2 (9 Feb 2026), copied unchanged from the official Windows
installer (mosquitto.org/download, `C:\Program Files\mosquitto`), so a
teammate on Windows needs only Python to run the dashboard.
`testing/subsystem1/s1_launch.py` looks here before an installed copy.

Used: `mosquitto.exe` (the broker), `mosquitto_sub.exe` and
`mosquitto_pub.exe` (the dashboard's clients) and their DLLs. The plugin
DLLs came along with the copy and are not loaded by the launcher's config.

The programs also need `VCRUNTIME140.dll`. Most PCs have it; when the
bundled copy is used, the launcher adds Python's own folder to the
children's PATH, and a python.org Python keeps one there.

Licence: mosquitto is dual-licensed under EPL-2.0 (`epl-v20`) and
EDL-1.0 (`edl-v10`); see `NOTICE.md`. OpenSSL (`libssl`, `libcrypto`) is
Apache-2.0.

To upgrade: install the new version on one PC, copy the same files here,
and change the version in this file and in the comment above
`BUNDLED_DIR` in `s1_launch.py`.
