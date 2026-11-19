# From scratch to the car on the dashboard (Windows)

For a teammate on Windows who has never built the car's firmware or run
the dashboard. Follow it top to bottom once; after that, only part E is
needed each session. Shorter reference: `docs/S1_quickstart.md`.

You end up with **two copies of the repository**:

| Copy | Where | Used for |
|---|---|---|
| **WSL copy** | `~/pico/INF2004_EMBEDDEDPROJECT_GRP13` inside Ubuntu | building the car's firmware (and your two config files) |
| **Windows copy** | `Documents\INF2004_EMBEDDEDPROJECT_GRP13` | running the dashboard |

Two copies because Git on Windows changes line endings, which can break
the Linux build, and the dashboard's `.bat` cannot run from inside WSL.

Every block below says which window to type it in: **PowerShell**,
**Admin PowerShell** (right-click Start > Terminal (Admin)) or
**Ubuntu** (the WSL terminal).

**Safety:** the broker has no login, so anyone who can reach it can
drive the car. Use only your own phone hotspot with a password only you
know. Never use school or public Wi-Fi. Never commit your two config
files, and never send anyone your `.uf2`: it contains your hotspot
password.

---

## A. Install once (Windows side)

1. **Python 3** from <https://www.python.org/downloads/> (not the
   Microsoft Store). Keep the default "py launcher" option ticked.
2. **Git for Windows** from <https://git-scm.com/download/win>
   (defaults are fine).
3. **WSL with Ubuntu.** Admin PowerShell:

   ```powershell
   wsl --install -d Ubuntu
   ```

   Restart the PC when asked. Ubuntu then opens and asks you to pick a
   Linux user name and password (remember the password: `sudo` asks for
   it).

Check Python. PowerShell:

```powershell
py -3 --version
```

It must print `Python 3.9` or later.

**Only if you ever installed mosquitto yourself:** stop its background
service, which would block port 1883. Admin PowerShell:

```powershell
Stop-Service mosquitto
```

```powershell
Set-Service mosquitto -StartupType Manual
```

(If it says the service does not exist, you never installed it: fine.)

## B. Get the repository (Windows copy, for the dashboard)

PowerShell:

```powershell
cd $HOME\Documents
```

```powershell
git clone https://github.com/YFArif/INF2004_EMBEDDEDPROJECT_GRP13.git
```

Nothing to configure in this copy. mosquitto is already inside it
(`tools\mosquitto-win`).

## C. Set up the firmware build (WSL copy)

Open **Ubuntu** for every command in this part.

**C.1 Tools:**

```bash
sudo apt update && sudo apt install -y git make g++ gcc-arm-none-eabi
```

**C.2 Pico SDK** (the `lib/lwip` submodule is needed for WiFi/MQTT):

```bash
mkdir -p ~/pico && cd ~/pico
```

```bash
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
```

```bash
cd ~/pico/pico-sdk && git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

**C.3 Tell the shell where the SDK is** (once):

```bash
echo 'export PICO_SDK_PATH="$HOME/pico/pico-sdk"' >> ~/.bashrc && source ~/.bashrc
```

Check both. The first must print a version, the second a list of files:

```bash
arm-none-eabi-gcc --version
```

```bash
ls $PICO_SDK_PATH
```

**C.4 The repository (WSL copy):**

```bash
cd ~/pico && git clone https://github.com/YFArif/INF2004_EMBEDDEDPROJECT_GRP13.git
```

```bash
cd ~/pico/INF2004_EMBEDDEDPROJECT_GRP13
```

## D. Your hotspot and the car's image (once, and again if they change)

**D.1 Phone hotspot:** 2.4 GHz and WPA2. iPhone: Personal Hotspot,
"Allow Others to Join" and "Maximize Compatibility" on. Android: band
2.4 GHz, security WPA2-Personal. The Pico W cannot join 5 GHz or
WPA3-only.

**D.2 Join the hotspot with the laptop.** Then make Windows treat it as
a Private network, or the car cannot reach the laptop. Admin
PowerShell:

```powershell
Get-NetConnectionProfile
```

If `NetworkCategory` says `Public`, run this with the `InterfaceAlias`
it shows (usually `Wi-Fi`):

```powershell
Set-NetConnectionProfile -InterfaceAlias "Wi-Fi" -NetworkCategory Private
```

**D.3 Find the laptop's address on the hotspot.** PowerShell:

```powershell
cd $HOME\Documents\INF2004_EMBEDDEDPROJECT_GRP13
```

```powershell
py -3 -I -B testing/subsystem1/s1_launch.py --show-address --no-browser
```

Write down the `this laptop's address: ...` line (something like
`172.20.10.4`), then press **Ctrl-C**. Windows may ask
about mosquitto here: tick **Private networks** only, never Public.
Do not paste the address into group chats.

**D.4 Your two config files.** Ubuntu:

```bash
cd ~/pico/INF2004_EMBEDDEDPROJECT_GRP13
```

```bash
cp config/wifi_secrets.example.h config/wifi_secrets.h
```

```bash
cp app_program/subsystem1/mqtt_config.example.h app_program/subsystem1/mqtt_config.h
```

Edit the first one:

```bash
nano config/wifi_secrets.h
```

Change only these two lines to your hotspot's name and password (leave
the `MQTT_*` lines below them alone; they are not used):

```c
#define WIFI_SSID     "your-hotspot-name"
#define WIFI_PASSWORD "your-hotspot-password"
```

Save with **Ctrl-O**, **Enter**, then exit with **Ctrl-X**. Then the
second one:

```bash
nano app_program/subsystem1/mqtt_config.h
```

Change these four lines (the other lines stay as they are). Put your
address from D.3 in place of `172.20.10.4`, and your buddy number in
the client ID:

```c
#define MQTT_LINK_CFG_SET               1
#define MQTT_LINK_CFG_BROKER_HOST       "172.20.10.4"
#define MQTT_LINK_CFG_CLIENT_ID         "robocar-buddy2"
#define MQTT_LINK_CFG_TOPIC_PREFIX      "robot"
```

- `MQTT_LINK_CFG_SET` must be `1`, or the build stops on purpose.
- The prefix must be exactly `"robot"` (the example says
  `"robocar/car1"`). With anything else the car connects but the
  dashboard stays empty.
- Leave port `1883U` and the empty username and password as they are.

Save and exit as before. Both files are git-ignored, so `git` will not
pick them up.

**D.5 Build.** Ubuntu:

```bash
cd ~/pico/INF2004_EMBEDDEDPROJECT_GRP13/build_make
```

```bash
make NET=lwip MQTT=1 -j8
```

It ends with a line naming `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`.
Do not add `TEST=1`: a test image ignores START and STOP.

**D.6 Flash.**

1. Car battery switch **off**.
2. Hold **BOOTSEL** on the Pico W, plug its USB cable into the laptop,
   then let go. A drive called **RPI-RP2** appears.
3. Open the build folder in File Explorer. Ubuntu (still in
   `build_make`):

   ```bash
   explorer.exe .
   ```

4. Drag `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2` onto the RPI-RP2
   drive. The Pico restarts by itself; a "not ejected properly" notice
   is harmless.

## E. Every session

1. Phone hotspot on. Laptop joined to it. Keep other devices off the
   hotspot.
2. Check the address has not changed. PowerShell:

   ```powershell
   cd $HOME\Documents\INF2004_EMBEDDEDPROJECT_GRP13
   ```

   ```powershell
   py -3 -I -B testing/subsystem1/s1_launch.py --show-address --no-browser
   ```

   Same address as in your `mqtt_config.h`? Ctrl-C and carry on. Changed?
   Redo D.4 (only the `BROKER_HOST` line), D.5 and D.6.
3. Start the dashboard: in File Explorer, open
   `Documents\INF2004_EMBEDDEDPROJECT_GRP13\testing\subsystem1` and
   double-click **`run_dashboard.bat`**. The browser opens
   http://127.0.0.1:8765/ by itself. Keep the black window open.
4. Switch the car on and wait for the robot light on the page to go
   green.
5. Your subsystem's data: click the **Developer** button for the S2,
   S3, S4 and S5 panels. Calibrate and run as in `docs/S1_handover.md`
   sections 4 and 5.
6. To stop: click the black window, press **Ctrl-C**, answer **Y**.
   Lines about mosquitto exiting at this point are normal.

No car with you? Practise with a fake car. PowerShell, in the Windows
copy:

```powershell
py -3 -I -B testing/subsystem1/s1_launch.py --simulate
```

## F. Getting new code

Windows copy (dashboard updates). PowerShell, with the dashboard
closed:

```powershell
cd $HOME\Documents\INF2004_EMBEDDEDPROJECT_GRP13
```

```powershell
git pull
```

WSL copy (firmware updates), then rebuild and reflash (D.5, D.6). Your
config files are kept. Ubuntu:

```bash
cd ~/pico/INF2004_EMBEDDEDPROJECT_GRP13 && git pull
```

There is one car. Whoever has it flashes their own image; the next
person reflashes with theirs.

## G. When something goes wrong

| What you see | Cause and fix |
|---|---|
| "port 1883 is already in use" | another mosquitto is running: `Stop-Service mosquitto` (Admin PowerShell), or close the other dashboard window |
| "port 8765 already answers" | a dashboard is already open: close its window (Ctrl-C) first |
| Page works, robot light never green | the address changed (E.2), the hotspot is Public (D.2), mosquitto was allowed on Public only (allow Private), or the car is on a different hotspot than in `wifi_secrets.h` |
| Light green, panels empty | `MQTT_LINK_CFG_TOPIC_PREFIX` is not `"robot"`: fix D.4, rebuild, reflash |
| `make`: "PICO_SDK_PATH is not set" | redo C.3, then open a new Ubuntu window |
| `make`: a missing `lwip` file or folder | run the submodule line in C.2 again |
| `make`: "mqtt_config.h: set the broker details" | `MQTT_LINK_CFG_SET` is still 0 (D.4) |
| car console says "authentication rejected" | hotspot is 5 GHz or WPA3, or the password is wrong; restart the hotspot with only the laptop and car on it |
| double-clicking the `.bat` says the py launcher was not found | install Python from python.org (A.1) |

Full details: `docs/S1_dashboard.md` (section 15 for Windows).
