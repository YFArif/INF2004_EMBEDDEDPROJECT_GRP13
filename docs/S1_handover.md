# S1 handover: WiFi, MQTT and the dashboard

One page for a teammate who has never run S1 (Buddy 1). Details live in
`docs/S1_dashboard.md` (every dashboard step, Windows in its section 15),
`docs/MQTT.md` (the link) and `testing/subsystem1/TEST_LOG.md` (what was
tested). Run every command from the repository root on macOS unless it
says otherwise. On Windows: build the firmware in WSL (`README.md`), and
run the broker and dashboard in PowerShell with `py -3` instead of
`python3` (`docs/S1_dashboard.md` section 15).

## The car today

It holds a credential-free image (`main` at `d63276e`, UF2 sha256
`fe372a1dab1bae82f5fc5088d280ed1f89edf14d240a5317ece1d82be1286298`): no
WiFi name, no password, no broker address. It cannot join any network
until you build and flash your own image as below.

## 1. Hotspot

- Only the broker laptop and the car connected. iPhone: Maximize
  Compatibility on (2.4 GHz), and keep the phone on the Personal Hotspot
  screen. Android: 2.4 GHz, WPA2-Personal.
- If the car's console loops `link status -> joined AP, waiting for IP
  (1)` then `FAILED: authentication rejected` (status 1 then -3), restart
  the hotspot with only those two devices on it.
- Never run the broker on a shared network: it accepts any client.

## 2. Build an image for your laptop

The broker address is baked in at build time. Join the broker laptop to
the hotspot first, then find its address (do not paste it anywhere):

```bash
ipconfig getifaddr en0
```

Make the two local config files (both git-ignored; never commit them,
never share the `.uf2` you build: it holds the WiFi password and the
broker address):

```bash
cp config/wifi_secrets.example.h config/wifi_secrets.h
```

```bash
cp app_program/subsystem1/mqtt_config.example.h app_program/subsystem1/mqtt_config.h
```

In `config/wifi_secrets.h` set `WIFI_SSID` and `WIFI_PASSWORD`. In
`mqtt_config.h` set `MQTT_LINK_CFG_SET` to `1`,
`MQTT_LINK_CFG_BROKER_HOST` to the address above and
`MQTT_LINK_CFG_TOPIC_PREFIX` to `"robot"` (`docs/S1_dashboard.md`
section 5). Then, with `PICO_SDK_PATH` set as in `README.md`:

```bash
cd build_make
```

```bash
make NET=lwip MQTT=1 -j8
```

Back to the repository root:

```bash
cd ..
```

**Flash (what worked on this car):** battery switch off, hold BOOTSEL
on the Pico W, plug the USB cable into the laptop, then release BOOTSEL;
the RPI-RP2 drive appears. From the repository root:

```bash
cp build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2 /Volumes/RPI-RP2/
```

The Pico restarts by itself. A "disk not ejected properly" notice is
harmless. If the hotspot address changes, rebuild and reflash.

## 3. A session

Hotspot on and the laptop joined first. Use one terminal window per
command that keeps running, each in the repository root.

**Broker.** Its config lives outside the repository
(`docs/S1_dashboard.md` section 6 explains each line). Leave the last
command running, and allow incoming connections if macOS asks; if
`mosquitto` is not found, Homebrew put it in `/opt/homebrew/sbin`.

```bash
mkdir -p ~/s1-broker
```

```bash
printf 'listener 1883\nallow_anonymous true\npersistence false\nsys_interval 5\n' > ~/s1-broker/mosquitto.conf
```

```bash
mosquitto -c ~/s1-broker/mosquitto.conf -v
```

**Dashboard.** Then open http://127.0.0.1:8765/:

```bash
python3 -I -B testing/subsystem1/s1_dashboard.py --host 127.0.0.1 --prefix robot
```

**Console** (S5's verdicts). With only one Pico plugged in, this waits
for the car's USB port, then prints the console until Ctrl-C:

```bash
until ls /dev | grep -q '^cu\.usbmodem'; do sleep 0.2; done; cat /dev/cu.usbmodem*
```

To catch the boot lines, keep this order: start the console command
first, then plug in USB, then switch the battery on. Then wait for the
green robot light.

**Demo day: the barcode letter map.** Type the day's letters in the
dashboard's map boxes (LEFT, RIGHT, STRAIGHT, U-TURN) and press Set map.
Set it after every power cycle of the car and wait for "Applied on the
robot: ..." before START. The dashboard sends it again by itself when the
car comes back, but check that it says Applied. Details:
`docs/S1_dashboard.md` section 8.2.

## 4. Calibrate before START

START is refused until the line sensors are calibrated. Put the car on
the line and, while it is READY (the dashboard's S5 panel), press Button
2 (GP21) and release it within 2 s; a press in any other state is
ignored. The car turns on the spot three times, and the console prints
`CAL line sensors calibrated: YES`. If it prints NO, press Button 2
again. Calibration is lost on every power-cycle. On the stand the sweep
returns NO (the sensors see no floor).

## 5. Reading the results

- An ack of **queued** means S1 handed the command to S5, not that it
  ran. S5's verdict and any refusal reason (for example `RUN refused:
  line sensors not calibrated`) appear on the console only.
- The console can drop bytes under load, and drops everything printed
  before it was opened. The dashboard's S5 panel (telemetry `ctrl_state`)
  is the reliable state source.
- Commands sent during an MQTT reconnect are dropped: they get no ack and
  never run. Resend if there is no ack.
- After a hotspot drop, telemetry resumed 12.1 s after the hotspot
  returned in the one 10 Nov 2026 test (n = 1). If the car is not back
  about a minute after the hotspot is back, power-cycle it (battery off,
  unplug USB for 2 s) and recalibrate.

## 6. Practice without the car

```bash
python3 -I -B testing/subsystem1/s1_dashboard.py --simulate
```

No broker and no car needed. The fake car does not model the
line-sensor check: it accepts START without calibration.

## 7. Known limits

- On 8 and 10 Nov 2026, NFR15 (a STOP acked under 300 ms in at least
  95 % of 50) was met in 2 of 6 bare-Pico runs and in 0 of 3 car runs
  (62, 40 and 64 %). The cause on the car is not established. To measure
  it: `python3 -I -B
  testing/subsystem1/cmd_rtt.py --prefix robot --op stop --count 50`
  (macOS or Linux only).
- On a phone hotspot telemetry arrives in bursts, and acks can lag by
  seconds.
- S5's verdict is not in the ack yet, and RSSI reads "not measured"
  (Stage 4b).
