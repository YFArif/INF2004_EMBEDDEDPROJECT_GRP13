# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `S2-U03`, `INT-01`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

All the clips below are chapters of **one unlisted YouTube video** (1080p,
no sound, 13:28): <https://www.youtube.com/watch?v=SKQWj1TgEVk>. Each link
jumps to the start of its test, where a 3-second title card names the test
and its ID. Only tests that need eyes
are filmed; automatic tests (`make TEST=1`, host tests) and timing
measurements are logs in each `testing/subsystemN/TEST_LOG.md`. "Demo test
case" numbers follow `SE13_Demo_Test_Cases.pdf`. *Date* is the date on the
clip file. *Commit* is `–` where the commit was not recorded when filming.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| 2026-11-25 | S1 | S1-U06 | – | 2502893 | [0:00](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=0s) | START, STOP and lift force stop sent from the dashboard over MQTT. Demo test cases #8, #9. |
| 2026-11-25 | S1 | S1-U17 | – | 2502893 | [0:43](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=43s) | S2 wheel speeds (mm/s) shown on the dashboard. Demo test case #9. |
| 2026-11-25 | S1 | S1-U17 | – | 2502893 | [0:51](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=51s) | S3's barcode result shown on the dashboard. Demo test case #9. |
| 2026-11-25 | S1 | S1-U17 | – | 2502893 | [1:11](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=71s) | S4 pitch and roll shown as numbers and as a graph. Demo test case #9. |
| 2026-11-25 | S1 | S1-U29 | – | 2502893 | [1:38](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=98s) | WiFi lost mid-run: Online, then Heartbeat Lost, then Offline; the car reconnects by itself. Demo test case #10. |
| 2026-11-25 | S2 | S2-U02 | – | 2501261 | [2:05](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=125s) | Speed control: from rest to 250 mm/s, then stop. |
| 2026-11-25 | S2 | S2-U03 | – | 2501261 | [2:16](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=136s) | Turn accuracy: 90 degrees left. |
| 2026-11-25 | S2 | S2-U03 | – | 2501261 | [2:25](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=145s) | Turn accuracy: 90 degrees right. |
| 2026-11-25 | S2 | S2-U03 | – | 2501261 | [2:34](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=154s) | Turn accuracy: 180-degree U-turn. |
| 2026-11-25 | S2 | S2-U05 | – | 2501261 | [2:45](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=165s) | Emergency brake: drives forward, then brakes and stops. |
| 2026-11-25 | S3 | S3-U02 | – | 2500316 | [2:56](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=176s) | Line following, one full lap; the dashboard shows the line state and steering. Demo test case #1. |
| 2026-11-25 | S3 | S3-U10, S3-U05, S3-U11 | – | 2500316 | [5:33](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=333s) | Barcode Z read; keymap says LEFT, so the car turns left at the next junction. Demo test cases #2, #3. |
| 2026-11-25 | S3 | S3-U10, S3-U05, S3-U11 | – | 2500316 | [6:30](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=390s) | Same barcode Z; keymap changed live to RIGHT, so the car turns right. Demo test cases #2, #3. |
| 2026-11-25 | S3 | S3-U10, S3-U05, S3-U11 | – | 2500316 | [7:24](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=444s) | Same barcode Z; keymap changed live to U-TURN, so the car turns around. Demo test cases #2, #3. |
| 2026-11-24 | S4 | S4-U01 | – | 2502114 | [8:16](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=496s) | Pitch on the dashboard against a protractor, about 20 degrees. |
| 2026-11-24 | S4 | S4-U02 | – | 2502114 | [8:38](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=518s) | Hump height, take 1: 10 mm hump, the dashboard shows about 10 mm. Demo test case #4. |
| 2026-11-24 | S4 | S4-U02 | – | 2502114 | [9:02](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=542s) | Hump height, take 2: 10 mm hump, the dashboard shows about 10 mm. Demo test case #4. |
| 2026-11-24 | S4 | S4-U10 | – | 2502114 | [9:22](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=562s) | Car lifted: the wheels stop. Demo test case #8. |
| 2026-11-25 | S5 | Demo: obstacle bypass | – | 2502447 | [9:35](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=575s) | Servo scan, bypass the box, rejoin the line. Demo test cases #5, #6, #7. |
| 2026-11-25 | S5 | Demo: side choice | – | 2502447 | [10:13](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=613s) | Left side blocked, so the car goes right. Demo test case #7. |
| 2026-11-25 | S5 | Demo: wider obstacle | – | 2502447 | [10:44](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=644s) | A box about 28 cm wide: width measured, steeper bypass. Demo test case #6. |
| 2026-11-26 | All | Demo: final mission | – | – | [11:23](https://www.youtube.com/watch?v=SKQWj1TgEVk&t=683s) | Full mission run, start to finish, all subsystems together. |
