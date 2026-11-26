# micro T-Kernel 3.0 project for Raspberry Pi Pico W

A starting point for your project: micro T-Kernel 3.0 running on **both cores**
of the Pico W's RP2040, with a USB serial console and the WiFi driver built in.

You need a **Raspberry Pi Pico W**, a USB cable, and a computer running
macOS, Linux, or Windows with **WSL (Ubuntu)**. On Windows, run every
command below inside the WSL Ubuntu terminal.

**Looking for the test videos or the coding standard?** Go to
[4. Test videos](#4-test-videos) (every test with a timestamped link) or
[5. Coding standard (BARR-C:2018)](#5-coding-standard-barr-c2018).

### Contents

1. [Set up the Pico C SDK and the compiler](#1-set-up-the-pico-c-sdk-and-the-compiler)
2. [Clone this repo and build the `.uf2` file](#2-clone-this-repo-and-build-the-uf2-file)
3. [Build options](#3-build-options-smp-console-wifi-test-crash_report)
4. [Test videos](#4-test-videos):
   [S1 Telemetry](#s1-telemetry-over-mqtt) ·
   [S2 Motion](#s2-motion-control) ·
   [S3 Line and barcode](#s3-line-following-and-barcode) ·
   [S4 IMU and terrain](#s4-imu-and-terrain) ·
   [S5 Obstacle avoidance](#s5-obstacle-avoidance) ·
   [Final mission](#final-mission)
5. [Coding standard (BARR-C:2018)](#5-coding-standard-barr-c2018)

---

## 1. Set up the Pico C SDK and the compiler

You need two separate things:

| What | Why | Where it comes from |
|---|---|---|
| **Pico C SDK** | USB and WiFi driver source code | `git clone` (step 1.2) |
| **Arm GNU Toolchain** (`arm-none-eabi-gcc`) | the compiler | a separate download (step 1.3) |

> ⚠️ **The SDK does NOT include the compiler.** You must install the compiler
> on its own and put its `bin` folder on your `PATH` (step 1.4).

**1.1 Install the basic tools: git, make and a host C++ compiler**

```sh
# macOS
xcode-select --install

# Linux / WSL (Ubuntu)
sudo apt update && sudo apt install -y git make g++
```

**1.2 Download the Pico C SDK** (any folder works; this guide uses `~/pico`)

```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver
```

The `submodule` line is required: it downloads the USB and WiFi drivers.
(SDK 2.3.0 has also been tested and works.)

**1.3 Install the Arm GNU Toolchain (the compiler)**

- **macOS / Windows (WSL) / Linux:** download from
  <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>.
  Pick the **AArch32 bare-metal target (arm-none-eabi)** package for your
  computer. For WSL, choose the **Linux x86_64** `.tar.xz`.
  Extract it, e.g. to `~/tools/arm-gnu-toolchain-<version>/`.
- **Linux / WSL shortcut:** `sudo apt install -y gcc-arm-none-eabi`
  installs it straight onto your PATH. If you use this, skip the PATH line in 1.4.

Find the folder that contains `arm-none-eabi-gcc`. It is the toolchain's
**`bin`** folder, e.g. `~/tools/arm-gnu-toolchain-15.3.rel1/bin`.

**1.4 Tell your shell where the compiler and SDK are**

Add these two lines to the end of `~/.zshrc` (macOS) or `~/.bashrc` (Linux/WSL),
**changing both paths to match your computer**:

```sh
export PATH="$HOME/tools/arm-gnu-toolchain-15.3.rel1/bin:$PATH"   # folder that contains arm-none-eabi-gcc
export PICO_SDK_PATH="$HOME/pico/pico-sdk"                         # the SDK from step 1.2
```

Open a **new** terminal, then check both:

```sh
arm-none-eabi-gcc --version     # must print a version, not "command not found"
ls $PICO_SDK_PATH               # must list the SDK files
```

If `arm-none-eabi-gcc` is "not found", your `PATH` does not point to the
toolchain's `bin` folder. Fix that before going further.

---

## 2. Clone this repo and build the `.uf2` file

**2.1 Clone your project**

```sh
cd ~/pico
git clone <URL-of-this-repo> my-project
cd my-project
```

**2.2 Build**

```sh
cd build_make
make -j8
```

A successful build ends with a line like
`tools/elf2uf2 mtk3pico_smp1_usb_cdc_wifi.elf mtk3pico_smp1_usb_cdc_wifi.uf2`.
Your firmware is **`build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`**.

If `make` stops with an error, it tells you which one is wrong:
`arm-none-eabi-gcc not found` means fix `PATH`, and
`PICO_SDK_PATH is not set` means fix `PICO_SDK_PATH` (step 1.4).

To build a single-core, UART-only or test version, see section 3.

**2.3 Flash it onto the Pico W**

1. Hold the **BOOTSEL** button and plug the Pico W into USB, then release.
   A drive called **RPI-RP2** appears.
2. Copy the `.uf2` file onto that drive. On WSL, run `explorer.exe .` in
   `build_make` to open the folder in Windows. The Pico reboots by itself.

**2.4 See the output**

Open the Pico's USB serial port at 115200 baud:

```sh
screen /dev/tty.usbmodem* 115200      # macOS   (quit: Ctrl-A then K)
screen /dev/ttyACM0 115200            # Linux
```

On Windows, use PuTTY or the VS Code *Serial Monitor* on the new COM port.
You should see the task reports. Connect an LED with a 330 Ω resistor from
**GP19** to GND to see subsystem 1's LED blink.

**2.5 Write your own code**

Your code lives in **`app_program/`**. Every `.c` file there, and in its
subfolders, is compiled automatically.

- **`app_main.c` → `usermain()` is where the program starts.** It first calls
  every subsystem's `subsystemN_init()` (create kernel objects), then every
  `subsystemN_start()` (start tasks). Read its comments first.
- **One folder per student:** `app_program/subsystem1/` … `subsystem5/`.
  Subsystems 1–4 hold a working example, one task each (LED, producer,
  consumer, monitor). Together they use the main micro T-Kernel APIs and
  pass data between subsystems. Subsystem 5 is an empty template.
  Replace the examples with your own subsystems.
- `subsystemN.h` is that subsystem's public interface: the object IDs and
  data formats the other subsystems may use.
- `app.h` holds the definitions shared by the whole team.
- **Drivers go in your subsystem folder too**, as a `drv_<name>.c` /
  `drv_<name>.h` pair, e.g. `subsystem2/drv_ultrasonic.c`. They are compiled
  automatically. Do not add drivers under `device/`: that folder holds the
  kernel's own serial, I2C and ADC drivers, and adding to it means editing
  kernel and makefile files shared by the whole team.

**Working as a team:** record owners, pins, priorities and the interfaces
between subsystems in **`TEAM.md`**, and record every test in **`testing/`**
(see `testing/README.md`). `make TEST=1` builds a test image that runs all
the subsystem tests at power-up. Short videos of tests and demos are shared
as unlisted YouTube links, listed in **`videos/`**.

Pins: use **GP0–GP22 and GP26–GP28**. GP0/GP1 are free unless your team
builds with `CRASH_REPORT=1` (section 3), which needs them. Never use
GP23/GP24/GP25/GP29: they are wired to the Pico W radio. The Pico W's
on-board LED is also on the radio, so use an external LED instead.

After editing, run `make -j8` again in `build_make` and copy the new `.uf2`.

---

## 3. Build options: `SMP`, `CONSOLE`, `WIFI`, `TEST`, `CRASH_REPORT`

`make` on its own uses the defaults. To build differently, put options
after `make`. They can be combined in any order:

```sh
make -j8                          # defaults: SMP=1 CONSOLE=usb_cdc WIFI=cyw43 TEST=0 CRASH_REPORT=0
make SMP=0 -j8                    # single-core build
make CONSOLE=uart WIFI=none -j8   # no USB console, radio off (no Pico SDK needed)
make TEST=1 -j8                   # firmware with the subsystem self-tests
make CRASH_REPORT=1 -j8           # also see crash reports, on GP0/GP1 (uses those pins)
```

To set several options at once, list them all, separated by spaces. For
example, to track down a crash in the test build while running on one core only:

```sh
make SMP=0 TEST=1 CRASH_REPORT=1 -j8
#  -> build_make/mtk3pico_smp0_usb_cdc_wifi_crash_test.uf2
```

Any option you leave out keeps its default (here `CONSOLE=usb_cdc` and
`WIFI=cyw43`). Options are not remembered: give them on every `make`, as
the next plain `make` goes back to the defaults.

| Option | Values (**default**) | What it does | Image name | C macro your code can test |
|---|---|---|---|---|
| `SMP` | `0`, **`1`** | `1` runs the kernel on both RP2040 cores; `0` uses core 0 only | `smp1` / `smp0` | `TK_SUPPORT_SMP` (1 or 0) |
| `CONSOLE` | **`usb_cdc`**, `uart` | where `tm_printf` output goes. `usb_cdc` = USB cable; `uart` = UART0 only (GP0 TX / GP1 RX, needs a USB-serial adapter) | `_usb_cdc` / `_uart` | `TM_CONSOLE_USB_CDC` |
| `WIFI` | **`cyw43`**, `none` | `cyw43` powers up the Pico W radio and its driver task; `none` leaves the radio off | `_wifi` / *(nothing)* | `TM_WIFI_CYW43` |
| `TEST` | **`0`**, `1` | `1` compiles every `subsystemN_test.c` and runs the tests at power-up | `_test` / *(nothing)* | `APP_TEST` (1 or 0) |
| `CRASH_REPORT` | **`0`**, `1` | `1` also copies the console to UART0 on **GP0 (TX) / GP1 (RX)**, 115200 baud, so you can see the last messages and the kernel's crash report (see below). **Uses GP0/GP1.** `CONSOLE=uart` always does this. | `_crash` / *(nothing)* | `TM_CONSOLE_UART` (1 or 0) |

**Why `CRASH_REPORT` exists.** USB output is not sent the moment you call
`tm_printf`: it waits in a buffer until the kernel's USB task sends it. If the
program crashes, no task runs again, so the last messages and the kernel's
crash report never reach your USB terminal. The terminal just goes quiet.
UART output *is* sent at once, even inside the crash handler. So when you
need to know why the Pico crashed or froze, build with `CRASH_REPORT=1`,
connect a USB-serial adapter (adapter RX → GP0, adapter GND → GND), and read
it at 115200 baud. Normal `tm_printf` output on USB works the same with
either setting. If a subsystem has claimed GP0/GP1 in `TEAM.md`, unplug that
hardware before using `CRASH_REPORT=1`.

The options are part of the image name, e.g.
`mtk3pico_smp1_usb_cdc_wifi_test.uf2`, so you can always tell which build
you are flashing. Switching options needs no `make clean`: the makefile
notices and recompiles everything.

Using an option in C code:

```c
#if TK_SUPPORT_SMP
    ctsk.tskatr |= TA_ASSPRC;   /* core affinity only exists on the dual-core build */
#endif

#if APP_TEST
    /* code that only exists in the test build */
#endif
```

### Where the options live in the makefile

All five are handled in **`build_make/pico_rp2040.mk`**. To find each piece, search the file for the
names in the left column (e.g. `grep -n "TEST" build_make/pico_rp2040.mk`).
Each option goes through the same steps:

| Step | What to look for | Example for `TEST` |
|---|---|---|
| 1. Default value | `NAME ?= value` near the top of the file | `TEST ?= 0` |
| 2. Allowed values | an `$(error Unknown ...)` check just below it | `$(error Unknown TEST '$(TEST)'; use 0 or 1)` |
| 3. Image-name suffix | `NAME_SUFFIX :=` and the `EXE_FILE :=` line | `TEST_SUFFIX := _test` |
| 4. C macro for the code | a `CFLAGS += -D...` line | `CFLAGS += -DAPP_TEST=$(TEST)` |
| 5. Rebuild on change | the `PROFILE_ID :=` line | `...-test$(TEST)` |
| 6. Extra source files (`CONSOLE`, `WIFI` only) | an `ifeq ($(CONSOLE),usb_cdc)` or `ifeq ($(WIFI),cyw43)` block that adds the USB/radio files and SDK include paths | — |

- **Changing a default:** edit the `?=` line from step 1, e.g.
  `CONSOLE ?= uart`. Every `make` without that option then uses the new value.
- **Adding your own option** (say `DEBUG=1` to turn on extra prints): copy
  how `TEST` is done. Add steps 1 to 5 next to the `TEST` lines, then
  use `#if APP_DEBUG` in your code. **Step 5 matters:** without it, switching
  the option can quietly link files that were compiled with the old setting.
- **Kernel settings** such as the maximum number of tasks, semaphores or
  mailboxes are not make options. They are in `config/config.h` (`CNF_MAX_...`).

The makefile also has `NET=lwip` and `MQTT=1` (TCP/IP and MQTT over WiFi).
They need the SDK's `lib/lwip` submodule and a `config/wifi_secrets.h`
(copy `config/wifi_secrets.example.h`); `MQTT=1` also needs
`app_program/subsystem1/mqtt_config.h` (copy `mqtt_config.example.h` in
the same folder). This guide does not cover them: see
`docs/S1_handover.md` and `docs/S1_dashboard.md` section 5.

One command starts the S1 broker and dashboard:
`python3 -I -B testing/subsystem1/s1_launch.py` (Windows: double-click
`testing\subsystem1\run_dashboard.bat`). First time? Start with
`docs/S1_quickstart.md`.

---

## 4. Test videos

All filmed tests are chapters of **one unlisted YouTube video** (1080p, no
sound, 13:28): <https://www.youtube.com/watch?v=SKQWj1TgEVk>.

- The tests are grouped by subsystem below. Click a test to open its
  description and a link that starts the video at that test.
- Each clip starts with a 3-second title card that names the test.
- [`videos/README.md`](videos/README.md) has the video index in the
  course's format, with the date and student ID for each test.
- The automatic test results are in `testing/subsystemN/TEST_LOG.md`.

### S1 Telemetry over MQTT

<details>
<summary><b>S1-U06</b>: START, STOP and lift force stop via MQTT</summary>

START and STOP are sent from the dashboard over MQTT and the car obeys.
Lifting the car triggers a force stop, which the dashboard reports. Demo
test cases #8 and #9.

Watch from [0:00](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=0s)

</details>

<details>
<summary><b>S1-U17</b>: Telemetry from the other subsystems reaches the dashboard</summary>

Each subsystem's values arrive on the dashboard: S2's wheel speeds in mm/s,
S3's barcode result, and S4's pitch and roll as numbers and as a graph. Demo
test case #9.

Watch: S2 wheel speeds from [0:43](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=43s), S3 barcode from [0:51](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=51s), S4 pitch and roll from [1:11](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=71s)

</details>

<details>
<summary><b>S1-U29</b>: WiFi lost mid-run, then reconnects</summary>

When WiFi is lost, the dashboard indicator goes from Online to Online
(Heartbeat Lost) for 3 s, then to Offline. When WiFi is back, the car
reconnects by itself and the indicator returns to Online. Demo test
case #10.

Watch from [1:38](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=98s)

</details>

### S2 Motion control

<details>
<summary><b>S2-U02</b>: Speed control, 250 mm/s</summary>

The robot starts from rest, drives forward at a target speed of 250 mm/s,
and stops. The target is both wheels at 225 to 275 mm/s within 0.5 s,
without large overshoot.

Watch from [2:05](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=125s)

</details>

<details>
<summary><b>S2-U03</b>: Turn accuracy</summary>

The robot turns from its starting direction and stops near the target angle:
90 degrees left, 90 degrees right, and a 180-degree U-turn.

Watch: Left 90 from [2:16](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=136s), Right 90 from [2:25](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=145s), U-turn 180 from [2:34](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=154s)

</details>

<details>
<summary><b>S2-U05</b>: Emergency brake</summary>

The robot drives forward, then brakes and stops when the emergency stop is
triggered.

Watch from [2:45](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=165s)

</details>

### S3 Line following and barcode

<details>
<summary><b>S3-U02</b>: Line following, one full lap</summary>

The car follows the line for a full lap and never leaves it. The dashboard
shows the line state (centred or correcting) and the steering, which change
as the car takes the curves. Demo test case #1.

Watch from [2:56](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=176s)

</details>

<details>
<summary><b>S3-U10, S3-U05, S3-U11</b>: Barcode Z sets the turn at the next junction</summary>

The car reads barcode Z while following the line, looks the letter up in the
keymap, and turns that way at the next junction. The same barcode is used in
all three runs; only the keymap was changed live on the dashboard, and the
car's turn changed with it. Demo test cases #2 and #3.

Watch: Z = LEFT from [5:33](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=333s), Z = RIGHT from [6:30](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=390s), Z = U-TURN from [7:24](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=444s)

</details>

### S4 IMU and terrain

<details>
<summary><b>S4-U01</b>: Pitch against a protractor</summary>

The car is tilted to about 20 degrees against a protractor, and the
dashboard shows the same pitch.

Watch from [8:16](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=496s)

</details>

<details>
<summary><b>S4-U02</b>: Hump height</summary>

The car drives over a 10 mm hump (measured with a ruler), and the dashboard
reports a height of about 10 mm. Two takes. Demo test case #4.

Watch: Take 1 from [8:38](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=518s), Take 2 from [9:02](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=542s)

</details>

<details>
<summary><b>S4-U10</b>: Car lifted: wheels stop</summary>

The car is lifted off the ground and the wheels stop. Demo test case #8.

Watch from [9:22](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=562s)

</details>

### S5 Obstacle avoidance

<details>
<summary><b>Demo</b>: Scan, bypass and rejoin the line</summary>

The car follows the line towards a box. After the servo scan, the dashboard
shows the box's width and the clear space on each side. The car goes round
the box, rejoins the line and carries on following it. Demo test
cases #5, #6 and #7.

Watch from [9:35](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=575s)

</details>

<details>
<summary><b>Demo</b>: Side choice: left blocked, goes right</summary>

A big box now blocks the left side and the right side is open. The car only
uses a side it fits through with a 7 cm margin, so it picks the right. Demo
test case #7.

Watch from [10:13](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=613s)

</details>

<details>
<summary><b>Demo</b>: Wider obstacle</summary>

A box about 28 cm wide, twice the first. The servo sweeps 40 degrees each
side and finds each edge in 5-degree steps; the width is the distance
between the edges. A wider box gets a steeper turn out and a longer move to
the side. Demo test case #6.

Watch from [10:44](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=644s)

</details>

### Final mission

<details>
<summary><b>Demo</b>: Full run, start to finish</summary>

The whole course with every subsystem working together.

Watch from [11:23](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=683s)

</details>

---

## 5. Coding standard (BARR-C:2018)

The firmware's C code follows the Barr Group **Embedded C Coding Standard,
BARR-C:2018**. That covers all 105 files in `app_program/`: the shared
files and subsystems S1 to S5.

- **Checked rule by rule.** Each subsystem was checked against every rule
  of the standard, with two scanners run over every build configuration
  and a review of each finding by hand.
- **No logic changed.** The changes are names, comments, white space,
  parentheses and layout. All 24 firmware configurations were built
  before and after, and every image is byte-for-byte identical. For the
  11 test builds, which print source line numbers, this holds with the
  line numbers fixed at 0.
- **Exceptions.** A few rules could only be met by changing what the code
  does, so the code stays as it is in those places. The main ones are 7
  functions longer than 100 lines (6.2.a), early `return`s (6.2.c,
  preferred practice), 10 places with `if`s nested three deep (8.2.b),
  four `continue`s (1.7.d), the test and logging macros (6.3.a) and the
  template's file layout (4.1.b, 4.2.a).
- **Abbreviations** (rule 1.5.b) are listed in `docs/abbreviations.md`
  for S1, and at the top of each `subsystemN.h` for S2 to S5.

The full report, with every exception and its reason, the readings
chosen where the standard leaves room, the byte-comparison table and the
old-to-new names of renamed kernel objects, is in
[`docs/barr_c_2018.md`](docs/barr_c_2018.md).

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
