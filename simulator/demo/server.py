#!/usr/bin/env python3
"""Local PX4-Lite / PX4 SITL comparison dashboard; stdlib only."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import socket
import subprocess
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


DEMO_DIR = Path(__file__).resolve().parent
REPO_ROOT = DEMO_DIR.parents[1]
DEFAULT_BINARY = REPO_ROOT / "build" / "simulator" / "px4lite_flight_demo.exe"
DEFAULT_PX4_PROJECT = Path(
    os.environ.get(
        "PX4_PROJECT_DIR",
        r"D:\px4Test" if os.name == "nt" else "/mnt/d/px4Test",
    )
)

COMBINED_BUSY_STATES = {
    "checking_px4",
    "starting_px4",
    "waiting_px4",
    "starting_lite",
    "running",
}


class Dashboard:
    def __init__(
        self, binary: Path, udp_host: str, udp_port: int, px4_project: Path
    ) -> None:
        self.binary = binary
        self.udp_host = udp_host
        self.udp_port = udp_port
        self.px4_project = px4_project
        self._lock = threading.RLock()
        self._states: dict[str, dict[str, Any]] = {}
        self._state_seen: dict[str, float] = {}
        self._lite_process: subprocess.Popen[str] | None = None
        self._lite_started_at: float | None = None
        self._lite_exit_code: int | None = None
        self._px4_process: subprocess.Popen[bytes] | None = None
        self._wsl_keepalive: subprocess.Popen[bytes] | None = None
        self._combined_status = "idle"
        self._combined_message = "請先啟動模擬環境，兩邊就緒後再按同步起飛。"
        self._launch_token = ""

    def start_udp_listener(self) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # A second Windows listener must not silently steal comparison packets.
        if os.name == "nt":
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        listener.bind((self.udp_host, self.udp_port))
        listener.settimeout(1.0)

        def receive() -> None:
            while True:
                try:
                    payload, _source = listener.recvfrom(8192)
                except socket.timeout:
                    continue
                except OSError:
                    return
                try:
                    state = json.loads(payload.decode("utf-8"))
                    vehicle = state.get("vehicle")
                    if vehicle not in ("px4", "px4lite"):
                        continue
                    with self._lock:
                        self._states[vehicle] = state
                        self._state_seen[vehicle] = time.monotonic()
                except (UnicodeDecodeError, json.JSONDecodeError, AttributeError):
                    continue

        threading.Thread(target=receive, name="px4-telemetry", daemon=True).start()

    def start_lite(self, from_combined: bool = False, wait_for_start: bool = False) -> tuple[bool, str]:
        with self._lock:
            if not from_combined and self._combined_status in COMBINED_BUSY_STATES:
                return False, "The synchronized start is already in progress."
            if self._lite_process is not None and self._lite_process.poll() is None:
                return True, "PX4-Lite flight is already running."
            if not self.binary.is_file():
                return False, f"Build the C++ demo first; executable not found: {self.binary}"
            if not from_combined:
                self._combined_status = "idle"
                self._combined_message = "PX4-Lite can run independently or through synchronized start."
            self._lite_process = subprocess.Popen(
                [str(self.binary), *(["--wait-for-start"] if wait_for_start else [])],
                cwd=REPO_ROOT,
                stdin=subprocess.PIPE if wait_for_start else subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
            )
            self._lite_started_at = time.monotonic()
            self._lite_exit_code = None
            self._states.pop("px4lite", None)
            self._state_seen.pop("px4lite", None)
            process = self._lite_process
            threading.Thread(
                target=self._read_lite_output,
                args=(process,),
                name="px4-lite-flight",
                daemon=True,
            ).start()
            return True, "PX4-Lite autonomous square flight started."

    def _platform_command(self, *args: str) -> list[str]:
        if os.name == "nt":
            # Docker is managed by the distro's system service. Avoid depending
            # on the default user's shell/session initialization (e.g. ROS).
            return ["wsl.exe", "-d", "Ubuntu", "--user", "root", "--exec", *args]
        return list(args)

    def _run_px4_command(
        self, *args: str, timeout: float = 60.0
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            self._platform_command(*args),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
            check=False,
        )

    def _px4_project_wsl_path(self) -> str:
        if os.name != "nt":
            return str(self.px4_project)
        drive = self.px4_project.drive.rstrip(":").lower()
        if not drive:
            raise ValueError(f"PX4 project path must be on a Windows drive: {self.px4_project}")
        return f"/mnt/{drive}{self.px4_project.as_posix()[2:]}"

    @staticmethod
    def _active_controller_check() -> str:
        return (
            "import glob; "
            "found=[]; "
            "\nfor p in glob.glob('/proc/[0-9]*/cmdline'): "
            "\n try: args=open(p,'rb').read().split(b'\\0') "
            "\n except OSError: continue "
            "\n if len(args)>2 and args[1:3] == [b'-u', b'/px4-tools/demo-flight.py']: found.append(p) "
            "\nprint('\\n'.join(found))"
        )

    def _prepare_px4_launch(self) -> tuple[dict[str, str] | None, str | None]:
        script = self.px4_project / "scripts" / "px4.sh"
        if not script.is_file():
            return None, f"PX4 launcher not found: {script}"

        try:
            if os.name == "nt":
                if self._wsl_keepalive is None or self._wsl_keepalive.poll() is not None:
                    self._wsl_keepalive = subprocess.Popen(
                        self._platform_command("/bin/sleep", "infinity"),
                        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        creationflags=subprocess.CREATE_NO_WINDOW,
                    )
                result = self._run_px4_command(
                    "bash", "-c", "systemctl start docker && docker ps -q "
                    "--filter label=com.docker.compose.project=px4-learning "
                    "--filter label=com.docker.compose.service=demo", timeout=120,
                )
            else:
                result = self._run_px4_command(
                    "docker", "ps", "-q", "--filter", "label=com.docker.compose.project=px4-learning",
                    "--filter", "label=com.docker.compose.service=demo",
                )
        except subprocess.TimeoutExpired:
            return None, "WSL／Docker 在 120 秒內未回應。請先開啟 Ubuntu 確認啟動完成，再重試。"
        except OSError as exc:
            return None, f"Could not query the WSL Docker engine: {exc}"
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            return None, f"Could not query the WSL Docker engine: {detail or 'docker ps failed'}"

        containers = [line.strip() for line in result.stdout.splitlines() if line.strip()]
        if len(containers) > 1:
            return None, "More than one PX4 demo container is running; stop the extras before synchronized start."
        if not containers:
            return {"kind": "compose", "container": ""}, None

        container = containers[0]
        with self._lock:
            state = self._states.get("px4")
            age = time.monotonic() - self._state_seen.get("px4", 0.0)
        if state is not None and age < 2.0:
            if state.get("ready_for_takeoff") and not state.get("armed"):
                return {"kind": "reuse", "container": container}, None
            return None, "PX4 SITL is already active. Let it land before starting a synchronized run."

        try:
            active = self._run_px4_command(
                "docker",
                "exec",
                container,
                "python3",
                "-c",
                self._active_controller_check(),
            )
            recent_log = self._run_px4_command(
                "docker", "exec", container, "sh", "-c", "tail -n 30 /px4-data/demo.log"
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return None, f"Could not inspect the existing PX4 session: {exc}"
        if active.returncode != 0 or active.stdout.strip():
            return None, "A PX4 flight controller is already running in the existing SITL container."
        latest_run = recent_log.stdout.rsplit("DEMO  Connecting to PX4 Demo MAVLink", 1)[-1]
        completed_landing = "DEMO  Demo complete" in latest_run
        failed_before_control = "DEMO ERROR  No PX4 Onboard MAVLink data received" in latest_run
        failed_before_control = failed_before_control or "DEMO  Demo aborted before arming" in latest_run
        if recent_log.returncode != 0 or not (completed_landing or failed_before_control):
            return None, "The existing PX4 session has not recorded a safe landing; finish or stop it first."
        if not failed_before_control and state is not None and not state.get("on_ground", False):
            return None, "The last PX4 telemetry shows the vehicle airborne; wait until it lands before restarting."
        return {"kind": "replace", "container": container}, None

    def _px4_launch_command(self, plan: dict[str, str]) -> list[str]:
        wsl_project = self._px4_project_wsl_path()
        prefix = ""
        if plan["kind"] == "replace":
            prefix = f"docker rm -f {shlex.quote(plan['container'])} >/dev/null && "
        command = prefix + (
            f"cd {shlex.quote(wsl_project)} && "
            f"PX4_COMPARE_PORT={self.udp_port} PX4_DEMO_NONINTERACTIVE=1 PX4_DEMO_WAIT_FOR_START=1 "
            "PX4_GUI=0 PX4_HEADLESS=1 ./scripts/px4.sh demo"
        )
        if os.name == "nt":
            return self._platform_command("bash", "-c", command)
        return ["bash", "-c", command]

    def _spawn_px4(self, plan: dict[str, str]) -> subprocess.Popen[bytes]:
        log_path = Path(os.environ.get("TEMP", "/tmp")) / "px4-lite-dashboard-px4.log"
        log_file = log_path.open("wb")
        kwargs: dict[str, Any] = {
            "cwd": REPO_ROOT,
            "stdin": subprocess.DEVNULL,
            "stdout": log_file,
            "stderr": subprocess.STDOUT,
            "close_fds": True,
        }
        if os.name == "nt":
            startup = subprocess.STARTUPINFO()
            startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            startup.wShowWindow = subprocess.SW_HIDE
            kwargs["startupinfo"] = startup
            kwargs["creationflags"] = subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
        try:
            process = subprocess.Popen(self._px4_launch_command(plan), **kwargs)
        finally:
            log_file.close()
        with self._lock:
            self._px4_process = process
        return process

    def prepare_environment(self) -> tuple[bool, str]:
        with self._lock:
            if self._combined_status in COMBINED_BUSY_STATES:
                return True, self._combined_message
            if self._lite_process is not None and self._lite_process.poll() is None:
                state = self._states.get("px4lite", {})
                if not state.get("ready_for_takeoff"):
                    return False, "PX4-Lite 正在飛行，請等待降落。"
            if not self.binary.is_file():
                return False, f"請先編譯 PX4-Lite：找不到 {self.binary}"
            self._combined_status = "checking_px4"
            self._combined_message = "正在啟動／檢查 WSL 和 Docker；冷啟動可能需要 1～2 分鐘…"

        threading.Thread(
            target=self._prepare_and_start_both,
            name="px4-combined-start",
            daemon=True,
        ).start()
        return True, self._combined_message

    def _prepare_and_start_both(self) -> None:
        try:
            self._prepare_and_start_both_checked()
        except Exception as exc:
            with self._lock:
                self._combined_status = "failed"
                self._combined_message = f"雙機啟動失敗：{exc}"

    def _prepare_and_start_both_checked(self) -> None:

        plan, error = self._prepare_px4_launch()
        if error is not None or plan is None:
            with self._lock:
                self._combined_status = "failed"
                self._combined_message = error or "Could not prepare PX4 SITL."
            return

        with self._lock:
            self._states.pop("px4", None)
            self._state_seen.pop("px4", None)
            self._combined_status = "starting_px4"
            if plan["kind"] == "replace":
                self._combined_message = "正在重新啟動已完成的 PX4 演示環境；啟動後將保持待命。"
            else:
                self._combined_message = "正在啟動 PX4／Gazebo 與 Lite，等待兩邊進入待命。"
        self._run_environment_start(plan)

    def _run_environment_start(self, plan: dict[str, str]) -> None:
        try:
            process = self._px4_process if plan["kind"] == "reuse" else self._spawn_px4(plan)
        except (OSError, ValueError) as exc:
            with self._lock:
                self._combined_status = "failed"
                self._combined_message = f"Could not start PX4 SITL: {exc}"
            return

        with self._lock:
            self._combined_status = "waiting_px4"
            self._combined_message = "正在等待 PX4／Gazebo 與 Lite 地面待命；此步驟不會起飛。"

        ok, message = self.start_lite(from_combined=True, wait_for_start=True)
        if not ok:
            self._fail(message)
            return

        ready_deadline = time.monotonic() + 150.0
        while time.monotonic() < ready_deadline:
            with self._lock:
                ready = self._vehicles_ready()
            if ready:
                break
            if process is not None and process.poll() is not None:
                with self._lock:
                    self._combined_status = "failed"
                    self._combined_message = "PX4 已退出，請檢查 PX4 啟動日誌。"
                return
            time.sleep(0.25)
        else:
            with self._lock:
                self._combined_status = "failed"
                self._combined_message = "模擬環境在 150 秒內未就緒，請檢查 PX4 與 Lite 日誌。"
            return

        with self._lock:
            self._combined_status = "ready"
            self._combined_message = "兩邊已就緒並保持地面待命，請按「同步起飛」。"

    def _vehicles_ready(self) -> bool:
        return all(
            time.monotonic() - self._state_seen.get(name, 0) < 2.0
            and self._states.get(name, {}).get("ready_for_takeoff", False)
            and not self._states.get(name, {}).get("armed", True)
            for name in ("px4", "px4lite")
        ) and self._lite_process is not None and self._lite_process.poll() is None

    def _fail(self, message: str) -> None:
        with self._lock:
            self._combined_status = "failed"
            self._combined_message = message

    def start_both(self) -> tuple[bool, str]:
        with self._lock:
            if self._combined_status == "running":
                return True, "雙機飛行已在執行。"
            if self._combined_status not in ("ready", "completed") or not self._vehicles_ready():
                return False, "請先啟動模擬環境，等待兩邊就緒再起飛。"
            token = uuid.uuid4().hex
            start_at = time.time() + 2.0
            trigger = self.px4_project / "data" / "gazebo" / "demo-start.json"
            temporary = trigger.with_suffix(".tmp")
            try:
                temporary.write_text(json.dumps({"token": token, "start_at": start_at}), encoding="utf-8")
                assert self._lite_process is not None and self._lite_process.stdin is not None
                self._lite_process.stdin.write(f"START {start_at:.6f}\n")
                self._lite_process.stdin.flush()
                temporary.replace(trigger)
            except (OSError, AssertionError) as exc:
                self.stop_lite()
                self._fail(f"起飛指令送出失敗：{exc}，請重新啟動環境。")
                return False, self._combined_message
            self._launch_token = token
            self._combined_status = "running"
            self._combined_message = "已送出雙機起飛指令，兩秒後開始飛行。"
        threading.Thread(target=self._watch_flight, args=(token,), daemon=True, name="px4-flight-monitor").start()
        return True, self._combined_message

    def _watch_flight(self, token: str) -> None:
        flight_deadline = time.monotonic() + 150.0
        while time.monotonic() < flight_deadline:
            with self._lock:
                lite_done = self._lite_exit_code is not None
                lite_exit = self._lite_exit_code
                px4_state = self._states.get("px4")
                process = self._px4_process
            px4_landed = bool(
                px4_state
                and px4_state.get("launch_token") == token
                and px4_state.get("ready_for_takeoff", False)
                and not px4_state.get("armed", True)
            )
            if lite_done and lite_exit != 0:
                self._fail(f"PX4-Lite 異常退出：{lite_exit}")
                return
            if lite_done and px4_landed:
                ok, message = self.start_lite(from_combined=True, wait_for_start=True)
                if not ok:
                    self._fail(message)
                    return
                for _ in range(40):
                    with self._lock:
                        if self._vehicles_ready():
                            break
                    time.sleep(0.1)
                else:
                    self._fail("兩台已降落，但 Lite 未重新進入待命狀態。")
                    return
                with self._lock:
                    self._combined_status = "completed"
                    self._combined_message = "兩台已降落並重新待命，可以再次按「同步起飛」。"
                return
            if process is not None and process.poll() is not None:
                with self._lock:
                    self._combined_status = "failed"
                    self._combined_message = "The PX4 demo process exited before both flights completed. Check the PX4 launcher log."
                return
            time.sleep(0.5)

        with self._lock:
            self._combined_status = "failed"
            self._combined_message = "The synchronized flight did not finish within 150 seconds."

    def _read_lite_output(self, process: subprocess.Popen[str]) -> None:
        assert process.stdout is not None
        for line in process.stdout:
            try:
                state = json.loads(line)
                if state.get("vehicle") != "px4lite":
                    continue
                with self._lock:
                    self._states["px4lite"] = state
                    self._state_seen["px4lite"] = time.monotonic()
            except json.JSONDecodeError:
                continue
        code = process.wait()
        with self._lock:
            if self._lite_process is process:
                self._lite_exit_code = code

    def stop_lite(self) -> tuple[bool, str]:
        with self._lock:
            process = self._lite_process
        if process is None or process.poll() is not None:
            return True, "PX4-Lite flight is already stopped."
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)
        with self._lock:
            self._lite_exit_code = process.returncode
        return True, "PX4-Lite flight stopped."

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            states = dict(self._states)
            seen = dict(self._state_seen)
            process = self._lite_process
            exit_code = self._lite_exit_code
            can_takeoff = self._combined_status in ("ready", "completed") and self._vehicles_ready()

        def endpoint(vehicle: str, key: str) -> dict[str, Any]:
            state = states.get(vehicle)
            age = now - seen.get(vehicle, 0.0)
            return {
                "connected": state is not None and age < 2.0,
                "age_s": age if state is not None else None,
                "state": state,
            }

        return {
            "lite": endpoint("px4lite", "px4lite"),
            "px4": endpoint("px4", "px4"),
            "lite_process_running": process is not None and process.poll() is None,
            "lite_exit_code": exit_code,
            "telemetry_udp_port": self.udp_port,
            "combined_start": {
                "status": self._combined_status,
                "message": self._combined_message,
                "can_takeoff": can_takeoff,
            },
        }


def make_handler(dashboard: Dashboard) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        server_version = "PX4LiteFlightDashboard/1.0"

        def _send(self, status: int, body: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, status: int, value: dict[str, Any]) -> None:
            self._send(status, json.dumps(value).encode("utf-8"), "application/json; charset=utf-8")

        def do_GET(self) -> None:
            if self.path == "/api/state":
                self._json(200, dashboard.snapshot())
                return
            if self.path in ("/", "/index.html"):
                path = DEMO_DIR / "viewer" / "index.html"
                self._send(200, path.read_bytes(), "text/html; charset=utf-8")
                return
            self._json(404, {"error": "not found"})

        def do_POST(self) -> None:
            if self.path == "/api/environment/start":
                ok, message = dashboard.prepare_environment()
                self._json(200 if ok else 409, {"ok": ok, "message": message})
                return
            if self.path == "/api/both/start":
                ok, message = dashboard.start_both()
                self._json(200 if ok else 409, {"ok": ok, "message": message})
                return
            if self.path == "/api/lite/start":
                ok, message = dashboard.start_lite()
                self._json(200 if ok else 409, {"ok": ok, "message": message})
                return
            if self.path == "/api/lite/stop":
                ok, message = dashboard.stop_lite()
                self._json(200 if ok else 409, {"ok": ok, "message": message})
                return
            if self.path == "/api/lite/reset":
                dashboard.stop_lite()
                ok, message = dashboard.start_lite()
                self._json(200 if ok else 409, {"ok": ok, "message": message})
                return
            self._json(404, {"error": "not found"})

        def log_message(self, fmt: str, *args: Any) -> None:
            print(f"[dashboard] {self.address_string()} {fmt % args}", flush=True)

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--http-host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8765)
    parser.add_argument("--udp-host", default="0.0.0.0")
    parser.add_argument("--udp-port", type=int, default=14552)
    parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    parser.add_argument("--px4-project", type=Path, default=DEFAULT_PX4_PROJECT)
    args = parser.parse_args()

    dashboard = Dashboard(args.binary.resolve(), args.udp_host, args.udp_port, args.px4_project)
    dashboard.start_udp_listener()
    server = ThreadingHTTPServer((args.http_host, args.http_port), make_handler(dashboard))
    print(
        f"PX4-Lite flight dashboard: http://{args.http_host}:{args.http_port} "
        f"(PX4 comparison telemetry UDP/{args.udp_port})",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.stop_lite()
        if dashboard._wsl_keepalive is not None:
            dashboard._wsl_keepalive.terminate()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
