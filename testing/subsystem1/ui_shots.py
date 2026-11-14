"""Screenshots of the S1 dashboard for the test log (S1-U26 part B).

Drives a headless Firefox over its Marionette port, standard library only.
For each shot it sets the viewport, the colour scheme and the accessibility
preferences in the browser (never in the page), loads the dashboard, waits
for live data and saves a PNG.

Run the dashboard in simulate mode first, then this script, both from the
repository root:

    python3 -I -B testing/subsystem1/s1_dashboard.py --simulate
    python3 -I -B testing/subsystem1/ui_shots.py --out testing/subsystem1/logs/ui

WARNING: only ever point this at a --simulate dashboard. A live page shows
the real topic prefix, and a screenshot of it would end up in the log.

Each file is <date>_<stage>_<name>.png. The script prints, for every shot,
what the page itself reports (viewport, media queries, font path), so a
preference that the browser did not apply is visible in the output.
"""

import argparse
import base64
import datetime
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time

FIREFOX_PATHS = (
    "/Applications/Firefox.app/Contents/MacOS/firefox",
    r"C:\Program Files\Mozilla Firefox\firefox.exe",
    "/usr/bin/firefox",
)
SETTLE_S = 4.0                  # Live data: two telemetry frames and more.
# name, width, height, scheme, extra preferences, page tweak
SHOTS = (
    ("light_1920x950", 1920, 950, "light", {}, ""),
    ("dark_1920x950", 1920, 950, "dark", {}, ""),
    ("light_1920x1080", 1920, 1080, "light", {}, ""),
    ("dark_1920x1080", 1920, 1080, "dark", {}, ""),
    ("light_1366x768", 1366, 768, "light", {}, ""),
    ("dark_1366x768", 1366, 768, "dark", {}, ""),
    ("light_375x812", 375, 812, "light", {}, ""),
    ("dark_375x812", 375, 812, "dark", {}, ""),
    ("light_contrast_more_1920x950", 1920, 950, "light",
     {"ui.useAccessibilityTheme": 1}, ""),
    ("dark_contrast_more_1920x950", 1920, 950, "dark",
     {"ui.useAccessibilityTheme": 1}, ""),
    ("light_reduced_transparency_1920x950", 1920, 950, "light",
     {"ui.prefersReducedTransparency": 1}, ""),
    ("light_reduced_motion_drawer_1920x950", 1920, 950, "light",
     {"ui.prefersReducedMotion": 1}, "drawer"),
    ("light_drawer_1920x950", 1920, 950, "light", {}, "drawer"),
    ("dark_drawer_1920x950", 1920, 950, "dark", {}, "drawer"),
    ("light_inter_forced_1920x950", 1920, 950, "light", {}, "inter"),
)
ALL_PREFS = ("ui.useAccessibilityTheme", "ui.prefersReducedTransparency",
             "ui.prefersReducedMotion")
SCHEME_PREF = "layout.css.prefers-color-scheme.content-override"
SCHEME_VALUE = {"dark": 0, "light": 1}
REPORT_JS = """
const q = (text) => matchMedia(text).matches;
const root = document.documentElement;
return {
  width: innerWidth, height: innerHeight,
  page_height: root.scrollHeight,
  dark: q("(prefers-color-scheme: dark)"),
  contrast_more: q("(prefers-contrast: more)"),
  forced_colors: q("(forced-colors: active)"),
  reduced_motion: q("(prefers-reduced-motion: reduce)"),
  reduced_transparency: q("(prefers-reduced-transparency: reduce)"),
  font: root.className,
  font_family: getComputedStyle(document.body).fontFamily,
  mode: document.getElementById("mode").textContent,
  link: document.getElementById("robot-text").textContent
};
"""
# Page tweaks are made from outside, after load, for one screenshot.
TWEAK_JS = {
    "": "return null;",
    "drawer": "document.getElementById('dev-open').click(); return null;",
    # Shows Inter and its tracking on a machine whose system font keywords
    # resolve. It does not exercise the detection in app.js.
    "inter": ("const r = document.documentElement;"
              "r.classList.remove('font-system');"
              "r.classList.add('font-inter');"
              "r.style.setProperty('--font-ui', 'Inter, sans-serif');"
              "return null;"),
}


class Marionette:
    """The few Marionette commands this script needs."""

    def __init__(self, port, timeout=30.0):
        deadline = time.monotonic() + timeout
        self.sock = None
        while self.sock is None:
            try:
                self.sock = socket.create_connection(("127.0.0.1", port),
                                                     timeout=60)
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.25)
        self.buffer = b""
        self.next_id = 0
        self._read()                    # The server's hello.
        self.call("WebDriver:NewSession", {})

    def _read(self):
        while b":" not in self.buffer:
            self.buffer += self._more()
        head, _, rest = self.buffer.partition(b":")
        size = int(head)
        while len(rest) < size:
            rest += self._more()
        self.buffer = rest[size:]
        return json.loads(rest[:size].decode("utf-8"))

    def _more(self):
        chunk = self.sock.recv(65536)
        if not chunk:
            raise RuntimeError("the browser closed the connection")
        return chunk

    def call(self, name, params):
        self.next_id += 1
        data = json.dumps([0, self.next_id, name, params]).encode("utf-8")
        self.sock.sendall(str(len(data)).encode("ascii") + b":" + data)
        reply = self._read()
        if reply[2] is not None:
            raise RuntimeError("%s: %s" % (name, reply[2].get("message")))
        return reply[3]

    def script(self, text, chrome=False):
        self.call("Marionette:SetContext",
                  {"value": "chrome" if chrome else "content"})
        result = self.call("WebDriver:ExecuteScript",
                           {"script": text, "args": []})
        self.call("Marionette:SetContext", {"value": "content"})
        return result.get("value")

    def set_pref(self, name, value):
        if value is None:
            self.script("Services.prefs.clearUserPref(%s);"
                        % json.dumps(name), chrome=True)
        else:
            self.script("Services.prefs.setIntPref(%s, %d);"
                        % (json.dumps(name), value), chrome=True)

    def viewport(self, width, height):
        """Size the window so that the page's viewport is width x height."""
        self._clamp("")
        self.call("WebDriver:SetWindowRect",
                  {"width": width, "height": height})
        got = None
        for _ in range(3):
            got = self.script("return [innerWidth, innerHeight];")
            if got == [width, height]:
                return
            rect = self.call("WebDriver:GetWindowRect", {})
            self.call("WebDriver:SetWindowRect",
                      {"width": rect["width"] + width - got[0],
                       "height": rect["height"] + height - got[1]})
        if got[0] > width:
            # A Firefox window has a minimum width (about 500 px), so a
            # phone width is made by narrowing the page's own frame.
            self._clamp("%dpx" % width)

    def _clamp(self, width):
        self.script("const b = gBrowser.selectedBrowser;"
                    "b.style.minWidth = b.style.maxWidth = %s;"
                    % json.dumps(width), chrome=True)

    def screenshot(self):
        value = self.call("WebDriver:TakeScreenshot",
                          {"full": False, "hash": False})["value"]
        return base64.b64decode(value)


def find_firefox(given):
    for path in ([given] if given else list(FIREFOX_PATHS)):
        if os.path.exists(path):
            return path
    sys.exit("ui_shots: Firefox not found; pass --firefox PATH")


def free_port():
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def take(browser, url, shot, out_dir, stage):
    name, width, height, scheme, prefs, tweak = shot
    browser.set_pref(SCHEME_PREF, SCHEME_VALUE[scheme])
    for pref in ALL_PREFS:
        browser.set_pref(pref, prefs.get(pref))
    browser.call("WebDriver:Navigate", {"url": "about:blank"})
    browser.viewport(width, height)
    browser.call("WebDriver:Navigate", {"url": url})
    time.sleep(SETTLE_S)
    browser.script(TWEAK_JS[tweak])
    time.sleep(0.6)
    report = browser.script(REPORT_JS)
    if "SIMULATED" not in report["mode"]:
        raise RuntimeError("not a --simulate dashboard: no screenshot taken")
    path = os.path.join(out_dir, "%s_%s_%s.png"
                        % (datetime.date.today().isoformat(), stage, name))
    with open(path, "wb") as handle:
        handle.write(browser.screenshot())
    print("%s: %s" % (os.path.basename(path), json.dumps(report,
                                                         sort_keys=True)),
          flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--url", default="http://127.0.0.1:8765/")
    parser.add_argument("--out", required=True, help="folder for the PNGs")
    parser.add_argument("--stage", default="stage1b",
                        help="word in each file name")
    parser.add_argument("--only", help="take only the shots whose name "
                        "contains this text")
    parser.add_argument("--firefox", help="path of the Firefox program")
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    profile = tempfile.mkdtemp(prefix="s1-ui-shots-")
    port = free_port()
    with open(os.path.join(profile, "user.js"), "w",
              encoding="utf-8") as handle:
        handle.write('user_pref("marionette.port", %d);\n' % port)
        # Firefox keeps this media query behind a preference.
        handle.write('user_pref("layout.css.prefers-reduced-transparency.'
                     'enabled", true);\n')
    proc = subprocess.Popen([find_firefox(args.firefox), "--headless",
                             "--no-remote", "--marionette",
                             "--remote-allow-system-access", "--profile",
                             profile], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    code = 0
    try:
        browser = Marionette(port)
        for shot in SHOTS:
            if (args.only is None) or (args.only in shot[0]):
                take(browser, args.url, shot, args.out, args.stage)
    except (RuntimeError, OSError) as err:
        print("ui_shots: stopped: %s" % err, file=sys.stderr)
        code = 1
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        shutil.rmtree(profile, ignore_errors=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
