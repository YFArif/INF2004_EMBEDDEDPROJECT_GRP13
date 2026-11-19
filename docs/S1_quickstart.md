# S1 dashboard quickstart

One page for a teammate who has never used the S1 dashboard. The full
guide is `docs/S1_dashboard.md`; the car side (calibration, START,
reading the results) is `docs/S1_handover.md`. Run every command from
the repository root. On Windows, write `py -3` wherever this page says
`python3`.

**The broker has no login.** Anyone who can reach it can send the car
commands. Run it only on your own phone hotspot (2.4 GHz, WPA2, a
password only you know), never on shared, school or public WiFi.

## 1. Install once

- **Python 3** (3.9 or later). macOS: from python.org or Homebrew.
  Windows: the python.org installer; check it with `py -3 --version`.
- **mosquitto** (2.0 or later). macOS: `brew install mosquitto`.
  **Windows: nothing to install.** A portable mosquitto 2.1.2 ships in
  `tools\mosquitto-win` and the launcher uses it. Install Python from
  python.org (not the Microsoft Store), because the bundled programs
  borrow its `vcruntime140.dll`.
- **Windows, only if you ran the mosquitto installer before: stop its
  service.** The installer starts a background broker that holds port
  1883. In an administrator PowerShell:

  ```powershell
  Stop-Service mosquitto
  ```

  ```powershell
  Set-Service mosquitto -StartupType Manual
  ```

- **Windows only: the firewall prompt.** The first time the broker
  starts, Windows asks about mosquitto. Allow **Private networks only**,
  never Public, and set your hotspot's network to Private
  (`docs/S1_dashboard.md` section 15.4).

## 2. Once per person: your settings

Two git-ignored files hold your settings. Never commit them and never
paste their values anywhere. Copy each from its example:

```bash
cp config/wifi_secrets.example.h config/wifi_secrets.h
```

```bash
cp app_program/subsystem1/mqtt_config.example.h app_program/subsystem1/mqtt_config.h
```

In `config/wifi_secrets.h`, set your hotspot (placeholders shown):

```c
#define WIFI_SSID     "your-hotspot-name"
#define WIFI_PASSWORD "your-hotspot-password"
```

In `app_program/subsystem1/mqtt_config.h` (placeholders shown;
`192.0.2.10` is a documentation address, use your laptop's):

```c
#define MQTT_LINK_CFG_SET               1
#define MQTT_LINK_CFG_BROKER_HOST       "192.0.2.10"
#define MQTT_LINK_CFG_BROKER_PORT       1883U
#define MQTT_LINK_CFG_CLIENT_ID         "robocar-buddy1"
#define MQTT_LINK_CFG_TOPIC_PREFIX      "robot"
#define MQTT_LINK_CFG_USERNAME          ""
#define MQTT_LINK_CFG_PASSWORD          ""
```

**The topic prefix must be exactly `"robot"`.** The example file sets
`MQTT_LINK_CFG_TOPIC_PREFIX` to `"robocar/car1"`, but the launcher and
the dashboard listen on `"robot"`. With any other prefix the car
connects and the page stays empty.

To find your laptop's address, join the hotspot, then run the launcher
with `--show-address`. Among its other lines it prints "this laptop's
address: ..."; copy that into `MQTT_LINK_CFG_BROKER_HOST` and press
Ctrl-C:

```bash
python3 -I -B testing/subsystem1/s1_launch.py --show-address
```

## 3. Build and flash the car's image

- Set up the Pico SDK and the compiler once, as in `README.md`
  section 1. On Windows, build inside WSL (Ubuntu), as that section
  says; the broker and the dashboard still run on Windows itself.
- Build with MQTT, from `build_make`:

  ```bash
  make NET=lwip MQTT=1 -j8
  ```

- Flash `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`: car
  battery off, hold BOOTSEL, plug in USB, copy the file onto the
  RPI-RP2 drive (`docs/S1_dashboard.md` section 5.4; `README.md`
  section 2.3 has the same for a bare Pico W).
- **Never share the `.uf2`:** it holds your WiFi password and your
  laptop's address.

## 4. Every session

1. Turn your own phone hotspot on and join it with the laptop before
   you start. School WiFi (WPA2-Enterprise) is not usable for the car,
   and the broker has no login.
2. Start the broker and the dashboard with one command:

   ```bash
   python3 -I -B testing/subsystem1/s1_launch.py
   ```

   On Windows, double-click `testing\subsystem1\run_dashboard.bat`
   instead.
3. The page opens on http://127.0.0.1:8765/. Switch the car on and wait
   for the green robot light, then carry on with `docs/S1_handover.md`.
4. Ctrl-C in the launcher's window stops the broker and the dashboard.
   On Windows the window then asks "Terminate batch job (Y/N)?": answer
   Y. Lines about the broker or `mosquitto_sub` exiting (an exit code such
   as 3221225786) may appear around this prompt; they are expected.

With the default config the broker log in the work folder
(`broker-<time>.log` in `~/s1-broker`) stays empty, because the config
has no `log_type`. To watch the broker's traffic, run this in a second
window:

```bash
mosquitto_sub -h 127.0.0.1 -t "#" -v
```

On Windows `mosquitto_sub` is in `tools\mosquitto-win` and is not on the
PATH. In PowerShell, from the repository root:

```powershell
.\tools\mosquitto-win\mosquitto_sub.exe -h 127.0.0.1 -t "#" -v
```

If the launcher says port 1883 is already in use, another broker is
running: on Windows it is usually the service (section 1). No car to
hand? `--simulate` starts a fake car and no broker.

## 5. When the address or the hotspot changes

The hotspot's name and password and your laptop's address are built
into the car's image. The phone can hand the laptop a new address when
either reconnects, so check it with `--show-address` before each
session. If anything changed: edit the two files, rebuild and reflash
(sections 2 and 3). The usual sign of a stale address is a robot light
that never turns green.

## 6. Windows: what has been checked

On 19 Nov 2026 the launcher ran on Windows 10 with Python 3.14 and the
bundled mosquitto. It started the broker on port 1883 on every
interface. A message published to the laptop's own network address
came back through the broker, the dashboard said "broker: connected",
and the page answered. Not yet checked: the double-click, Ctrl-C in the
window, and a real car connecting through the Windows firewall. That
full run is S1-U23 part C. If the launcher fails, `docs/S1_dashboard.md`
section 15 has the manual steps, which are the fallback.

If the car never connects (the robot light never turns green) while the
page works, the usual cause is the firewall: the hotspot is set as a
Public network, or mosquitto was allowed on Public only. Fix it as in
section 1.
