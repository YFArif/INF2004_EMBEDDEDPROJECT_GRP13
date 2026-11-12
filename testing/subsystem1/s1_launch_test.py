"""S1-U23 part A: host test of s1_launch.py's helpers (macOS or Linux).

Run by run_launcher_test.sh, or from the repository root:

    python3 -I -B testing/subsystem1/s1_launch_test.py

--module-dir DIR tests the s1_launch.py in DIR instead (the runner's
mutated copies). --no-e2e skips the end-to-end --simulate runs, which need
s1_dashboard.py next to the launcher and a free port on 127.0.0.1.
Everything happens in a temporary folder: the real ~/s1-broker is never
touched. The last line is "S1-U23 part A (host): PASS (...)" with exit
status 0, or "... FAIL (...)" with exit status 1.
"""

import argparse
import datetime
import importlib.util
import os
import queue
import re
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
IPV4 = re.compile(r"(?<![\d.])(?:\d{1,3}\.){3}\d{1,3}(?![\d.])")
TOOLS = ("mosquitto", "mosquitto_sub", "mosquitto_pub")


class Checks:
    def __init__(self):
        self.passed = 0
        self.failed = []

    def check(self, name, b_ok, detail=""):
        print("%s %s%s" % (name, "PASS" if b_ok else "FAIL",
                           (": " + detail) if detail else ""), flush=True)
        if b_ok:
            self.passed += 1
        else:
            self.failed.append(name)


def load(module_dir):
    path = os.path.join(module_dir, "s1_launch.py")
    spec = importlib.util.spec_from_file_location("s1_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, path


def stub_tools(folder, names):
    os.makedirs(folder, exist_ok=True)
    for name in names:
        path = os.path.join(folder, name)
        with open(path, "w", encoding="ascii") as stub:
            stub.write("#!/bin/sh\nexit 0\n")
        os.chmod(path, 0o755)


def free_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def test_tools(c, launch, tmp):
    on_path = os.path.join(tmp, "path_bin")
    fallback = os.path.join(tmp, "fallback_bin")
    empty = os.path.join(tmp, "empty_bin")
    os.makedirs(empty)
    stub_tools(on_path, TOOLS)
    stub_tools(fallback, TOOLS)
    found, extra, missing = launch.find_tools(path=on_path,
                                              fallback=[fallback])
    c.check("L1", (sorted(found) == sorted(TOOLS)) and (extra == [])
            and (missing == [])
            and all(p.startswith(on_path) for p in found.values()),
            "all three on the PATH, no folder added")
    found, extra, missing = launch.find_tools(path=empty,
                                              fallback=[empty, fallback])
    c.check("L2", (sorted(found) == sorted(TOOLS)) and (extra == [fallback])
            and (missing == []),
            "found only in a fallback folder, which is then added")
    partial = os.path.join(tmp, "partial_bin")
    stub_tools(partial, ("mosquitto", "mosquitto_pub"))
    found, extra, missing = launch.find_tools(path=empty,
                                              fallback=[partial])
    text = launch.missing_text(missing, windows=False)
    wtext = launch.missing_text(missing, windows=True)
    c.check("L3", (missing == ["mosquitto_sub"])
            and ("mosquitto_sub" in text) and ("brew install" in text)
            and ("C:\\Program Files\\mosquitto" in wtext),
            "one missing: named, with an install hint")
    base = {"PATH": "/a" + os.pathsep + "/b", "KEEP": "1"}
    before = os.environ.get("PATH")
    env = launch.child_env([fallback], base=base)
    c.check("L4", (env["PATH"] == os.pathsep.join([fallback, "/a", "/b"]))
            and (env["KEEP"] == "1") and (base["PATH"] == "/a:/b")
            and (os.environ.get("PATH") == before),
            "children's PATH gets the folder; the launcher's does not")
    c.check("L5", (launch.fallback_dirs(True)
                   == [r"C:\Program Files\mosquitto"])
            and ("/opt/homebrew/sbin" in launch.fallback_dirs(False)),
            "fallback folders for Windows and macOS")


def test_config(c, launch, tmp):
    work = os.path.join(tmp, "work")
    path, b_new = launch.ensure_config(work)
    with open(path, encoding="ascii") as conf:
        first = conf.read()
    c.check("L6", b_new and (first == "listener 1883\nallow_anonymous true\n"
                             "persistence false\nsys_interval 5\n"),
            "created with the four lines")
    path2, b_new2 = launch.ensure_config(work)
    with open(path, "a", encoding="ascii") as conf:
        conf.write("# changed by hand\n")
    path3, b_new3 = launch.ensure_config(work)
    with open(path, encoding="ascii") as conf:
        last = conf.read()
    c.check("L7", (not b_new2) and (not b_new3) and (path2 == path)
            and (path3 == path) and last.endswith("# changed by hand\n"),
            "never created twice, a changed file is left as it is")


def test_ports(c, launch):
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    port = listener.getsockname()[1]
    b_busy = launch.port_in_use(port)
    listener.close()
    time.sleep(0.2)
    b_free = not launch.port_in_use(port)
    c.check("L8", b_busy and b_free, "dummy listener seen, then gone")
    broker = launch.busy_text(1883, 8765)
    page = launch.busy_text(8765, 8765)
    c.check("L9", ("Stop-Service mosquitto" in broker)
            and ("\"mosquitto\"" in broker)
            and ("another dashboard" in page)
            and ("missing or wrong page token" in page),
            "the two refusal texts")

    def fake_run(out):
        def run(*args, **kwargs):
            return subprocess.CompletedProcess(args, 0, stdout=out)
        return run

    c.check("L10", launch.service_running(True, fake_run(
        "STATE              : 4  RUNNING"))
            and (not launch.service_running(True, fake_run(
                "[SC] OpenService FAILED 1060")))
            and (not launch.service_running(False, fake_run("RUNNING"))),
            "sc query: RUNNING warns, a missing service or macOS does not")


def test_names(c, launch, tmp):
    when = launch.stamp(datetime.datetime(2026, 11, 12, 9, 5, 7))
    folder = os.path.join(tmp, "names")
    os.makedirs(folder)
    first = launch.unique_path(folder, "run", when, ".jsonl")
    open(first, "w").close()
    second = launch.unique_path(folder, "run", when, ".jsonl")
    open(second, "w").close()
    third = launch.unique_path(folder, "run", when, ".jsonl")
    c.check("L11", (when == "20261112-090507")
            and first.endswith("run-20261112-090507.jsonl")
            and second.endswith("run-20261112-090507-2.jsonl")
            and third.endswith("run-20261112-090507-3.jsonl")
            and launch.unique_path(folder, "broker", when, ".log")
            .endswith("broker-20261112-090507.log"),
            "timestamped names, no clash in one second")


def test_args(c, launch):
    own, extra = launch.split_args(["--prefix", "car", "--", "--banner-off",
                                    "--ack-timeout", "3"])
    args, _ = launch.parse(own)
    argv = launch.dashboard_argv("py", args.prefix, "L.jsonl", 8766, False,
                                 extra)
    sim = launch.dashboard_argv("py", "robot", "L.jsonl", 8765, True, [])
    c.check("L12", (own == ["--prefix", "car"])
            and (argv[:3] == ["py", "-I", "-B"])
            and argv[3].endswith("s1_dashboard.py")
            and (argv[4:] == ["--host", "127.0.0.1", "--prefix", "car",
                              "--log", "L.jsonl", "--http-port", "8766",
                              "--banner-off", "--ack-timeout", "3"])
            and ("--simulate" not in argv) and (sim[-1] == "--simulate")
            and (launch.broker_argv("m", "c.conf") == ["m", "-c", "c.conf"]),
            "dashboard arguments, pass-through, --simulate")
    args, extra = launch.parse([])
    c.check("L13", (args.prefix == "robot") and (args.http_port == 8765)
            and (not args.simulate) and (not args.show_address)
            and (not args.no_browser) and (extra == []),
            "defaults: robot, 8765, no simulate, no address")


def test_log_check(c, launch, tmp):
    empty = os.path.join(tmp, "empty.jsonl")
    open(empty, "w").close()
    full = os.path.join(tmp, "full.jsonl")
    with open(full, "w") as log:
        log.write("{}\n")
    gone = os.path.join(tmp, "gone.jsonl")
    e_lines = launch.log_lines(empty, "test")
    g_lines = launch.log_lines(gone, "test")
    f_lines = launch.log_lines(full, "test")
    c.check("L14", any("EMPTY" in line for line in e_lines)
            and any("EMPTY" in line for line in g_lines)
            and (not any("EMPTY" in line for line in f_lines))
            and ("3 bytes" in f_lines[0]),
            "0 bytes or no file warns, a log with data does not")
    home = os.path.expanduser("~")
    c.check("L15", (launch.display_path(os.path.join(home, "s1-broker"),
                                        windows=False)
                    == "~" + os.sep + "s1-broker")
            and launch.display_path(os.path.join(home, "x"), windows=True)
            .startswith("%USERPROFILE%")
            and (launch.display_path("/elsewhere/x") == "/elsewhere/x"),
            "home folder shown as ~ or %USERPROFILE%")


def test_children(c, launch):
    stubborn = subprocess.Popen(
        [sys.executable, "-I", "-B", "-c",
         "import signal, time; "
         "signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)"])
    time.sleep(0.5)
    start = time.monotonic()
    code = launch.stop_child(stubborn, wait_s=1.0)
    took = time.monotonic() - start
    c.check("L16", (code == -signal.SIGKILL) and (took < 3.0),
            "a child that ignores SIGTERM is killed after the wait")
    c.check("L17", (launch.exit_code(0) == 0) and (launch.exit_code(1) == 1)
            and (launch.exit_code(-15) == 143)
            and (launch.exit_code(None) == 1),
            "exit codes, 128 + n after a signal")


def read_lines(stream, out):
    for line in iter(stream.readline, ""):
        out.put(line)
    out.put(None)


def launch_proc(script, args, env=None, b_session=False):
    proc = subprocess.Popen([sys.executable, "-I", "-B", script] + args,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, env=env,
                            start_new_session=b_session)
    lines = queue.Queue()
    threading.Thread(target=read_lines, args=(proc.stdout, lines),
                     daemon=True).start()
    return proc, lines


def collect(lines, until, limit_s):
    """Lines until one contains `until` (or the stream ends, or time)."""
    got = []
    end = time.monotonic() + limit_s
    while time.monotonic() < end:
        try:
            line = lines.get(timeout=0.2)
        except queue.Empty:
            continue
        if line is None:
            break
        got.append(line.rstrip("\n"))
        if (until is not None) and (until in line):
            break
    return got


def no_address(lines):
    return all(found == "127.0.0.1" for line in lines
               for found in IPV4.findall(line))


def test_refusals(c, script, tmp):
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    port = listener.getsockname()[1]
    work = os.path.join(tmp, "refuse")
    proc, lines = launch_proc(script, ["--simulate", "--no-browser",
                                       "--workdir", work,
                                       "--http-port", str(port)])
    rc = proc.wait(timeout=30)
    out = collect(lines, None, 2.0)
    listener.close()
    c.check("L18", (rc == 4) and any("another dashboard" in line
                                     for line in out)
            and (not any("dashboard: http" in line for line in out)),
            "page port taken: refused, exit 4, nothing started")
    empty = os.path.join(tmp, "empty_bin2")
    os.makedirs(empty)
    env = dict(os.environ, PATH=empty)
    proc, lines = launch_proc(script, ["--no-browser", "--workdir", work,
                                       "--fallback-dir", empty], env=env)
    rc = proc.wait(timeout=30)
    out = collect(lines, None, 2.0)
    c.check("L19", (rc == 2) and any(
        ("mosquitto, mosquitto_sub, mosquitto_pub not found" in line)
        for line in out),
            "no mosquitto anywhere: named, exit 2")


def test_simulate_run(c, script, tmp):
    work = os.path.join(tmp, "sim")
    port = free_port()
    proc, lines = launch_proc(script, ["--simulate", "--no-browser",
                                       "--workdir", work,
                                       "--http-port", str(port)])
    out = collect(lines, "Ctrl-C to quit", 20.0)
    start = time.monotonic()
    while (time.monotonic() - start < 15.0) and (not _answers(port)):
        time.sleep(0.2)
    b_page = _answers(port)
    out += collect(lines, "15 s after the start", 25.0)
    proc.send_signal(signal.SIGINT)
    rc = proc.wait(timeout=20)
    out += collect(lines, None, 3.0)
    logs = [name for name in os.listdir(work) if name.startswith("run-")]
    size = os.path.getsize(os.path.join(work, logs[0])) if logs else 0
    printed = [line for line in out if line.startswith("s1_launch: ")]
    c.check("L20", b_page and (rc == 0) and (size > 0)
            and any("15 s after the start" in line for line in printed)
            and any("(at exit)" in line for line in printed)
            and (not any("EMPTY" in line for line in printed))
            and any("broker log: none (--simulate)" in line
                    for line in printed),
            "--simulate run: page answers, log %d bytes, exit %s"
            % (size, rc))
    c.check("L21", no_address(out) and (not any(
        name.startswith("broker-") for name in os.listdir(work))),
            "no address printed, no broker log in --simulate")
    port = free_port()
    proc, lines = launch_proc(script, ["--simulate", "--no-browser",
                                       "--workdir", work,
                                       "--http-port", str(port)],
                              b_session=True)
    collect(lines, "Ctrl-C to quit", 20.0)
    start = time.monotonic()
    while (time.monotonic() - start < 15.0) and (not _answers(port)):
        time.sleep(0.2)
    os.killpg(proc.pid, signal.SIGINT)     # What Ctrl-C in a terminal does.
    rc = proc.wait(timeout=20)
    time.sleep(1.0)
    try:
        os.killpg(proc.pid, 0)
        b_left = True
    except ProcessLookupError:
        b_left = False
    c.check("L22", (rc == 0) and (not b_left) and (not _answers(port)),
            "Ctrl-C to the whole group: exit %s, nothing left" % rc)


def _answers(port):
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.5):
            return True
    except OSError:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--module-dir", default=HERE)
    parser.add_argument("--no-e2e", action="store_true")
    args = parser.parse_args()
    launch, script = load(args.module_dir)
    c = Checks()
    with tempfile.TemporaryDirectory(prefix="s1u23.") as tmp:
        test_tools(c, launch, tmp)
        test_config(c, launch, tmp)
        test_ports(c, launch)
        test_names(c, launch, tmp)
        test_args(c, launch)
        test_log_check(c, launch, tmp)
        test_children(c, launch)
        if not args.no_e2e:
            test_refusals(c, script, tmp)
            test_simulate_run(c, script, tmp)
    verdict = "FAIL" if c.failed else "PASS"
    print("S1-U23 part A (host): %s (%d passed%s)"
          % (verdict, c.passed,
             (", failed: " + " ".join(c.failed)) if c.failed else ""),
          flush=True)
    return 1 if c.failed else 0


if __name__ == "__main__":
    sys.exit(main())
