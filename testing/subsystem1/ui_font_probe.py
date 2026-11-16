"""Which font the S1 dashboard uses in Chromium, per platform (S1-U26 B).

Starts a headless Chromium-family browser, tells it to report itself as
Windows and then as macOS (DevTools protocol, standard library only), loads
the dashboard each time and prints what the page did: the class on <html>,
whether the Inter file was requested, and what document.fonts reports.

Run the dashboard in simulate mode first, then this script, both from the
repository root:

    python3 -I -B testing/subsystem1/s1_dashboard.py --simulate
    python3 -I -B testing/subsystem1/ui_font_probe.py

Exit status 0: on Windows the page is html.font-inter, requested the Inter
file and document.fonts has Inter loaded; on macOS it is html.font-system
and did not request the file. Otherwise 1.
"""

import argparse
import base64
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request

BROWSER_PATHS = (
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    "/Applications/Brave Browser.app/Contents/MacOS/Brave Browser",
    "/Applications/Microsoft Edge.app/Contents/MacOS/Microsoft Edge",
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    "/usr/bin/chromium",
)
FONT_PATH = "/fonts/inter-latin-opsz-normal.woff2"
SETTLE_S = 3.0
PLATFORMS = {
    "windows": ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/"
                "537.36 (KHTML, like Gecko) Chrome/130.0.0.0 Safari/537.36",
                "Win32", "Windows"),
    "macos": ("Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/"
              "537.36 (KHTML, like Gecko) Chrome/130.0.0.0 Safari/537.36",
              "MacIntel", "macOS"),
}
REPORT_JS = """(async () => {
  await document.fonts.ready;
  const inter = [...document.fonts].filter((f) => f.family.includes("Inter"));
  return JSON.stringify({
    html_class: document.documentElement.className,
    platform: navigator.platform,
    ua_platform: navigator.userAgentData
      ? navigator.userAgentData.platform : null,
    font_family: getComputedStyle(document.body).fontFamily,
    inter_faces: inter.map((f) => f.status),
    inter_check: document.fonts.check('600 17px "Inter"'),
    mode: document.getElementById("mode").textContent
  });
})()"""


class DevTools:
    """A DevTools protocol connection: one WebSocket, text frames only."""

    def __init__(self, ws_url):
        host_port, _, path = ws_url[len("ws://"):].partition("/")
        host, _, port = host_port.partition(":")
        self.sock = socket.create_connection((host, int(port)), timeout=30)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        self.sock.sendall((
            "GET /%s HTTP/1.1\r\nHost: %s\r\nUpgrade: websocket\r\n"
            "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
            % (path, host_port, key)).encode("ascii"))
        self.buffer = b""
        while b"\r\n\r\n" not in self.buffer:
            self.buffer += self._more()
        head, _, self.buffer = self.buffer.partition(b"\r\n\r\n")
        if b" 101 " not in head.split(b"\r\n")[0]:
            raise RuntimeError("no WebSocket upgrade")
        self.next_id = 0
        self.events = []

    def _more(self):
        chunk = self.sock.recv(65536)
        if not chunk:
            raise RuntimeError("the browser closed the connection")
        return chunk

    def _take(self, count):
        while len(self.buffer) < count:
            self.buffer += self._more()
        data, self.buffer = self.buffer[:count], self.buffer[count:]
        return data

    def _send(self, text):
        data = text.encode("utf-8")
        mask = os.urandom(4)
        if len(data) < 126:
            head = struct.pack("!BB", 0x81, 0x80 | len(data))
        elif len(data) < 65536:
            head = struct.pack("!BBH", 0x81, 0x80 | 126, len(data))
        else:
            head = struct.pack("!BBQ", 0x81, 0x80 | 127, len(data))
        body = bytes(byte ^ mask[i % 4] for i, byte in enumerate(data))
        self.sock.sendall(head + mask + body)

    def _receive(self):
        """One whole text message (fragments joined)."""
        message = b""
        while True:
            first, second = self._take(2)
            size = second & 0x7F
            if size == 126:
                size = struct.unpack("!H", self._take(2))[0]
            elif size == 127:
                size = struct.unpack("!Q", self._take(8))[0]
            payload = self._take(size)
            if (first & 0x0F) in (0x0, 0x1):
                message += payload
                if first & 0x80:
                    return json.loads(message.decode("utf-8"))

    def call(self, method, params=None):
        self.next_id += 1
        self._send(json.dumps({"id": self.next_id, "method": method,
                               "params": params or {}}))
        while True:
            reply = self._receive()
            if reply.get("id") == self.next_id:
                if "error" in reply:
                    raise RuntimeError("%s: %s" % (method, reply["error"]))
                return reply.get("result", {})
            self.events.append(reply)

    def drain(self, seconds):
        """Collect events for a while."""
        self.sock.settimeout(0.2)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                self.events.append(self._receive())
            except socket.timeout:
                pass
        self.sock.settimeout(30)


def find_browser(given):
    for path in ([given] if given else list(BROWSER_PATHS)):
        if os.path.exists(path):
            return path
    sys.exit("ui_font_probe: no Chromium-family browser found; pass "
             "--browser PATH")


def free_port():
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def page_socket(port):
    """The WebSocket URL of the browser's first page."""
    deadline = time.monotonic() + 30
    while True:
        try:
            with urllib.request.urlopen("http://127.0.0.1:%d/json/list"
                                        % port, timeout=2) as reply:
                pages = [page for page in json.load(reply)
                         if page.get("type") == "page"]
            if pages:
                return pages[0]["webSocketDebuggerUrl"]
        except OSError:
            pass
        if time.monotonic() > deadline:
            raise RuntimeError("the browser did not start")
        time.sleep(0.25)


def probe(tools, url, name):
    agent, platform, ua_platform = PLATFORMS[name]
    tools.call("Emulation.setUserAgentOverride", {
        "userAgent": agent, "platform": platform,
        "userAgentMetadata": {"platform": ua_platform,
                              "platformVersion": "10.0.0",
                              "architecture": "x86", "model": "",
                              "mobile": False}})
    tools.call("Network.setCacheDisabled", {"cacheDisabled": True})
    tools.events = []
    tools.call("Page.navigate", {"url": url})
    tools.drain(SETTLE_S)
    asked = [event["params"]["request"]["url"] for event in tools.events
             if event.get("method") == "Network.requestWillBeSent"]
    result = tools.call("Runtime.evaluate", {"expression": REPORT_JS,
                                             "awaitPromise": True})
    report = json.loads(result["result"]["value"])
    report["inter_requested"] = any(FONT_PATH in item for item in asked)
    if "SIMULATED" not in report.pop("mode"):
        raise RuntimeError("not a --simulate dashboard")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--url", default="http://127.0.0.1:8765/")
    parser.add_argument("--browser", help="path of the browser program")
    args = parser.parse_args()
    profile = tempfile.mkdtemp(prefix="s1-font-probe-")
    port = free_port()
    proc = subprocess.Popen([find_browser(args.browser), "--headless=new",
                             "--remote-debugging-port=%d" % port,
                             "--user-data-dir=" + profile, "--no-first-run",
                             "about:blank"], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    code = 1
    try:
        tools = DevTools(page_socket(port))
        tools.call("Network.enable")
        tools.call("Page.enable")
        win = probe(tools, args.url, "windows")
        mac = probe(tools, args.url, "macos")
        print("windows: %s" % json.dumps(win, sort_keys=True))
        print("macos:   %s" % json.dumps(mac, sort_keys=True))
        b_win = (("font-inter" in win["html_class"])
                 and win["inter_requested"] and win["inter_check"]
                 and ("loaded" in win["inter_faces"]))
        b_mac = (("font-system" in mac["html_class"])
                 and (not mac["inter_requested"]))
        print("Windows uses Inter, requested and loaded: %s; macOS uses "
              "the system font, Inter not requested: %s" % (b_win, b_mac))
        code = 0 if (b_win and b_mac) else 1
    except (RuntimeError, OSError, KeyError, ValueError) as err:
        print("ui_font_probe: stopped: %s" % err, file=sys.stderr)
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
