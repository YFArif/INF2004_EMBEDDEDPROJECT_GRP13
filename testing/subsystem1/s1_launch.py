"""S1 launcher: the broker and the dashboard with one command (S1-U23).

Finds mosquitto, mosquitto_sub and mosquitto_pub, keeps the broker's
mosquitto.conf in a work folder outside the repository (~/s1-broker, on
Windows %USERPROFILE%\\s1-broker), refuses to start when the broker port
(1883) or the page port (8765) already answers, then runs the broker and
s1_dashboard.py as child processes and opens the page in the browser.
Every dashboard event goes to a new log in the work folder
(run-<time>.jsonl); the launcher warns if that log is still empty 15 s
after the start, and again at exit. Ctrl-C, or either child stopping,
stops both: the broker is never left running.

Run from the repository root:

    python3 -I -B testing/subsystem1/s1_launch.py
    py -3 -I -B testing/subsystem1/s1_launch.py          (Windows)

or double-click testing\\subsystem1\\run_dashboard.bat on Windows.
--simulate starts no broker and needs no mosquitto (a fake car answers).
Anything after "--" goes to s1_dashboard.py unchanged.

It prints the page's address, the two log paths and "Ctrl-C to quit",
never this laptop's address, the WiFi name or a login. --show-address
prints the laptop's main IPv4 address, the one mqtt_config.h needs; it is
found with a UDP socket that is connected but sends nothing. Standard
library only; no network use beyond 127.0.0.1.

Exit status: the dashboard's, or 2 (a mosquitto program is missing),
3 (the broker did not start), 4 (a port is already in use).
"""

import argparse
import datetime
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
import webbrowser

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DASHBOARD = os.path.join(HERE, "s1_dashboard.py")

IS_WINDOWS = (os.name == "nt")
LOOPBACK = "127.0.0.1"
BROKER_PORT = 1883
HTTP_PORT = 8765
TOOLS = ("mosquitto", "mosquitto_sub", "mosquitto_pub")
WINDOWS_DIRS = [r"C:\Program Files\mosquitto"]
MAC_DIRS = ["/opt/homebrew/sbin", "/opt/homebrew/bin", "/usr/local/sbin",
            "/usr/local/bin"]
CONFIG_NAME = "mosquitto.conf"
CONFIG_TEXT = ("listener 1883\n"
               "allow_anonymous true\n"
               "persistence false\n"
               "sys_interval 5\n")
BROKER_WAIT_S = 5.0
PAGE_WAIT_S = 15.0
LOG_CHECK_S = 15.0
STOP_WAIT_S = 3.0
POLL_S = 0.2
TAIL_LINES = 10
NO_ROUTE = ("192.0.2.1", 9)     # TEST-NET-1: connect() only picks a route.
EXIT_NO_TOOL = 2
EXIT_NO_BROKER = 3
EXIT_PORT_BUSY = 4
WINDOWS_NOTE = ("s1_launch: if Windows asks about mosquitto, allow Private "
                "networks only.")


def say(text):
    print("s1_launch: " + text, flush=True)


def fallback_dirs(windows=IS_WINDOWS):
    return list(WINDOWS_DIRS if windows else MAC_DIRS)


def find_tools(path=None, fallback=None, windows=IS_WINDOWS):
    """Return ({tool: full path}, [folders to add to PATH], [missing]).

    The PATH (or path, when given) comes first, then each fallback folder.
    """
    if fallback is None:
        fallback = fallback_dirs(windows)
    found = {}
    extra = []
    missing = []
    for tool in TOOLS:
        where = shutil.which(tool, path=path)
        if where is None:
            for folder in fallback:
                where = shutil.which(tool, path=folder)
                if where is not None:
                    if folder not in extra:
                        extra.append(folder)
                    break
        if where is None:
            missing.append(tool)
        else:
            found[tool] = where
    return found, extra, missing


def install_hint(windows=IS_WINDOWS):
    if windows:
        return ("install mosquitto from mosquitto.org/download (it goes to "
                "C:\\Program Files\\mosquitto)")
    return "install it with: brew install mosquitto"


def missing_text(missing, windows=IS_WINDOWS):
    return "%s not found: %s" % (", ".join(missing), install_hint(windows))


def child_env(extra, base=None):
    """A copy of the environment with extra folders at the front of PATH.

    Only the children get it: the launcher's own PATH is not changed.
    """
    env = dict(os.environ if base is None else base)
    if extra:
        env["PATH"] = os.pathsep.join(list(extra) + [env.get("PATH", "")])
    return env


def default_workdir():
    return os.path.join(os.path.expanduser("~"), "s1-broker")


def ensure_config(workdir):
    """Create workdir/mosquitto.conf if it is missing; never overwrite it.

    Returns (path, True if it was created now).
    """
    os.makedirs(workdir, exist_ok=True)
    path = os.path.join(workdir, CONFIG_NAME)
    try:
        with open(path, "x", encoding="ascii", newline="\n") as conf:
            conf.write(CONFIG_TEXT)
    except FileExistsError:
        return path, False
    return path, True


def port_in_use(port, host=LOOPBACK, timeout=0.5):
    """True if something accepts a TCP connection on host:port."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def busy_text(port, http_port):
    if port == http_port:
        return ("port %d already answers: another dashboard is running. "
                "Close it first (on Windows two dashboards can share a "
                "port, and the page then says \"missing or wrong page "
                "token\"), or use --http-port." % port)
    return ("port %d is already in use, so the broker cannot start. "
            "Usually it is the Windows service \"mosquitto\": stop it in an "
            "administrator PowerShell with: Stop-Service mosquitto. On "
            "macOS or Linux, quit the other mosquitto first." % port)


def service_running(windows=IS_WINDOWS, run=subprocess.run):
    """True if Windows reports the mosquitto service as RUNNING."""
    if not windows:
        return False
    try:
        out = run(["sc", "query", "mosquitto"], capture_output=True,
                  text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        return False
    return "RUNNING" in out


def stamp(now=None):
    return (now or datetime.datetime.now()).strftime("%Y%m%d-%H%M%S")


def unique_path(folder, prefix, when, ext):
    """folder/prefix-when.ext, or with -2, -3 ... if that name exists."""
    path = os.path.join(folder, "%s-%s%s" % (prefix, when, ext))
    number = 2
    while os.path.exists(path):
        path = os.path.join(folder, "%s-%s-%d%s" % (prefix, when, number,
                                                    ext))
        number += 1
    return path


def split_args(argv):
    """Split at the first "--": (launcher's own, the dashboard's)."""
    if "--" in argv:
        cut = argv.index("--")
        return argv[:cut], argv[cut + 1:]
    return argv, []


def dashboard_argv(python, prefix, log_path, http_port, simulate, extra):
    argv = [python, "-I", "-B", DASHBOARD, "--host", LOOPBACK,
            "--prefix", prefix, "--log", log_path,
            "--http-port", str(http_port)]
    if simulate:
        argv.append("--simulate")
    return argv + list(extra)


def broker_argv(mosquitto, conf_path):
    return [mosquitto, "-c", conf_path]


def wait_for_port(port, proc, limit_s):
    """True once port answers; False if proc exits first or time runs out."""
    end = time.monotonic() + limit_s
    while time.monotonic() < end:
        if port_in_use(port):
            return True
        if proc.poll() is not None:
            return False
        time.sleep(POLL_S)
    return False


def tail(path, count=TAIL_LINES):
    try:
        with open(path, encoding="utf-8", errors="replace") as handle:
            return handle.read().splitlines()[-count:]
    except OSError:
        return []


def log_size(path):
    try:
        return os.path.getsize(path)
    except OSError:
        return 0


def log_lines(path, when):
    """The log's size at `when`, and a warning if nothing was captured."""
    size = log_size(path)
    lines = ["log %s: %d bytes (%s)" % (display_path(path), size, when)]
    if 0 == size:
        lines.append("WARNING: the dashboard log is EMPTY: nothing was "
                     "captured. Is the page you are using the one this "
                     "launcher opened?")
    return lines


def display_path(path, windows=IS_WINDOWS):
    """The path with the home folder as ~ (Windows: %USERPROFILE%), so
    no user name is shown."""
    home = os.path.expanduser("~")
    if (path == home) or path.startswith(home + os.sep):
        return ("%USERPROFILE%" if windows else "~") + path[len(home):]
    return path


def primary_ipv4():
    """This laptop's main IPv4 address; connect() on UDP sends nothing."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(NO_ROUTE)
            address = probe.getsockname()[0]
    except OSError:
        return None
    return None if address.startswith("0.") else address


def stop_child(proc, wait_s=STOP_WAIT_S, b_own_exit=False):
    """Terminate, wait, then kill. Returns the exit code.

    b_own_exit: Ctrl-C reached the child too (a terminal sends it to the
    whole process group, a Windows console to every process in it), so
    the child first gets wait_s to finish its own shutdown; a terminate
    in the middle of that would cut it short.
    """
    if proc is None:
        return None
    if b_own_exit and (proc.poll() is None):
        try:
            proc.wait(timeout=wait_s)
        except subprocess.TimeoutExpired:
            pass
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=wait_s)
        except subprocess.TimeoutExpired:
            proc.kill()
    return proc.wait()


def exit_code(returncode):
    """A child's code as a shell sees it: 128 + n if signal n ended it."""
    if returncode is None:
        return 1
    return returncode if returncode >= 0 else 128 - returncode


def parse(argv):
    own, extra = split_args(argv)
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        epilog="Anything after -- goes to s1_dashboard.py unchanged.")
    parser.add_argument("--prefix", default="robot", help="topic prefix")
    parser.add_argument("--workdir", default=None,
                        help="broker config and logs (default ~/s1-broker)")
    parser.add_argument("--http-port", type=int, default=HTTP_PORT,
                        help="port of the page on 127.0.0.1")
    parser.add_argument("--no-browser", action="store_true",
                        help="do not open the page")
    parser.add_argument("--simulate", action="store_true",
                        help="no broker: the dashboard's fake car answers")
    parser.add_argument("--show-address", action="store_true",
                        help="print this laptop's main IPv4 address")
    parser.add_argument("--fallback-dir", action="append",
                        help=argparse.SUPPRESS)
    return parser.parse_args(own), extra


def preflight(args):
    """The tools, the work folder and the ports; exits on a problem."""
    if IS_WINDOWS:
        print(WINDOWS_NOTE, flush=True)
    found, extra, missing = find_tools(fallback=args.fallback_dir)
    if missing and not args.simulate:
        say(missing_text(missing))
        sys.exit(EXIT_NO_TOOL)
    workdir = os.path.abspath(args.workdir or default_workdir())
    conf, _ = ensure_config(workdir)
    ports = [args.http_port] if args.simulate else [BROKER_PORT,
                                                    args.http_port]
    busy = [port for port in ports if port_in_use(port)]
    for port in busy:
        say(busy_text(port, args.http_port))
    if (not args.simulate) and service_running():
        say("WARNING: the Windows service \"mosquitto\" is RUNNING. Stop "
            "it in an administrator PowerShell: Stop-Service mosquitto")
    if busy:
        sys.exit(EXIT_PORT_BUSY)
    return found, extra, workdir, conf


def start_broker(found, env, workdir, conf, when):
    """Start mosquitto; exits 3 with its last log lines if it fails."""
    log_path = unique_path(workdir, "broker", when, ".log")
    with open(log_path, "w", encoding="utf-8") as out:
        proc = subprocess.Popen(broker_argv(found["mosquitto"], conf),
                                stdout=out, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, env=env,
                                cwd=workdir)
    if not wait_for_port(BROKER_PORT, proc, BROKER_WAIT_S):
        stop_child(proc)
        say("the broker did not start; the last lines of %s:"
            % display_path(log_path))
        for line in tail(log_path):
            print("    " + line, flush=True)
        sys.exit(EXIT_NO_BROKER)
    return proc, log_path


def watch(dash, broker, args, log_path):
    """Open the page, check the log once, and wait for a child to stop."""
    url = "http://%s:%d/" % (LOOPBACK, args.http_port)
    start = time.monotonic()
    b_opened = args.no_browser
    b_checked = False
    while True:
        if dash.poll() is not None:
            return
        if (broker is not None) and (broker.poll() is not None):
            say("the broker stopped; stopping the dashboard")
            return
        elapsed = time.monotonic() - start
        if (not b_opened) and port_in_use(args.http_port):
            webbrowser.open(url)
            b_opened = True
        elif (not b_opened) and (elapsed > PAGE_WAIT_S):
            say("the page did not answer within %d s" % PAGE_WAIT_S)
            b_opened = True
        if (not b_checked) and (elapsed >= LOG_CHECK_S):
            for line in log_lines(log_path, "15 s after the start"):
                say(line)
            b_checked = True
        time.sleep(POLL_S)


def on_signal(signum, frame):
    raise KeyboardInterrupt


def main(argv=None):
    args, extra = parse(sys.argv[1:] if argv is None else argv)
    found, dirs, workdir, conf = preflight(args)
    env = child_env(dirs)
    when = stamp()
    log_path = unique_path(workdir, "run", when, ".jsonl")
    for name in ("SIGTERM", "SIGHUP"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), on_signal)
    broker = None
    broker_log = None
    dash = None
    b_ctrl_c = False
    try:
        if not args.simulate:
            broker, broker_log = start_broker(found, env, workdir, conf,
                                              when)
        dash = subprocess.Popen(
            dashboard_argv(sys.executable, args.prefix, log_path,
                           args.http_port, args.simulate, extra),
            env=env, cwd=REPO, stdin=subprocess.DEVNULL)
        say("dashboard: http://%s:%d/" % (LOOPBACK, args.http_port))
        say("log: %s" % display_path(log_path))
        say("broker log: %s" % ("none (--simulate)" if broker_log is None
                                else display_path(broker_log)))
        if args.show_address:
            say("this laptop's address: %s"
                % (primary_ipv4() or "none (no network)"))
        say("Ctrl-C to quit")
        watch(dash, broker, args, log_path)
    except KeyboardInterrupt:
        b_ctrl_c = True
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        code = stop_child(dash, b_own_exit=b_ctrl_c)
        stop_child(broker, b_own_exit=b_ctrl_c)
        if dash is not None:
            for line in log_lines(log_path, "at exit"):
                say(line)
    return exit_code(code)


if __name__ == "__main__":
    sys.exit(main())
