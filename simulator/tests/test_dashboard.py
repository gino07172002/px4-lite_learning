"""Regression checks for the web launcher's slow WSL startup path."""

import tempfile
import threading
import time
import unittest
import io
import json
import subprocess
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch

from simulator.demo.server import Dashboard


class DashboardLaunchTest(unittest.TestCase):
    def test_slow_preflight_does_not_block_http_or_launch_twice(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "demo"
            binary.touch()
            dashboard = Dashboard(binary, "127.0.0.1", 0, Path(directory))
            entered, release, finished = (threading.Event() for _ in range(3))

            def preflight():
                entered.set()
                release.wait(3)
                return None, "WSL startup timed out"

            def worker():
                try:
                    dashboard._prepare_and_start_both_checked()
                finally:
                    finished.set()

            with patch.object(dashboard, "_prepare_px4_launch", side_effect=preflight) as check:
                with patch.object(dashboard, "_prepare_and_start_both", side_effect=worker):
                    started = time.monotonic()
                    self.assertTrue(dashboard.prepare_environment()[0])
                    self.assertLess(time.monotonic() - started, 0.5)
                    self.assertTrue(entered.wait(1))
                    self.assertTrue(dashboard.prepare_environment()[0])
                    self.assertEqual(check.call_count, 1)
                    self.assertEqual(dashboard.snapshot()["combined_start"]["status"], "checking_px4")
                    release.set()
                    self.assertTrue(finished.wait(1))
                    self.assertEqual(dashboard.snapshot()["combined_start"], {
                        "status": "failed", "message": "WSL startup timed out", "can_takeoff": False,
                    })

    def test_missing_binary_never_starts_px4(self):
        with tempfile.TemporaryDirectory() as directory:
            dashboard = Dashboard(Path(directory) / "missing", "127.0.0.1", 0, Path(directory))
            with patch.object(dashboard, "_prepare_px4_launch") as check:
                self.assertFalse(dashboard.prepare_environment()[0])
                check.assert_not_called()

    def test_unexpected_launch_error_is_reported_in_state(self):
        dashboard = Dashboard(Path("unused"), "127.0.0.1", 0, Path("unused"))
        with patch.object(dashboard, "_prepare_px4_launch", side_effect=RuntimeError("launch failed")):
            dashboard._prepare_and_start_both()
        state = dashboard.snapshot()["combined_start"]
        self.assertEqual(state["status"], "failed")
        self.assertIn("launch failed", state["message"])

    def test_takeoff_requires_prepared_live_vehicles(self):
        with tempfile.TemporaryDirectory() as directory:
            dashboard = Dashboard(Path(directory) / "demo", "127.0.0.1", 0, Path(directory))
            with patch.object(dashboard, "_spawn_px4") as spawn:
                self.assertFalse(dashboard.start_both()[0])
                spawn.assert_not_called()
            self.assertFalse((Path(directory) / "data/gazebo/demo-start.json").exists())

    def test_takeoff_uses_one_shared_deadline_without_restarting_environment(self):
        with tempfile.TemporaryDirectory() as directory:
            project = Path(directory)
            (project / "data/gazebo").mkdir(parents=True)
            dashboard = Dashboard(project / "demo", "127.0.0.1", 0, project)
            stream = io.StringIO()
            dashboard._lite_process = SimpleNamespace(poll=lambda: None, stdin=stream)
            dashboard._combined_status = "ready"
            for name in ("px4", "px4lite"):
                dashboard._states[name] = {"ready_for_takeoff": True, "armed": False}
                dashboard._state_seen[name] = time.monotonic()
            with patch.object(dashboard, "_watch_flight"), patch.object(dashboard, "_spawn_px4") as spawn:
                self.assertTrue(dashboard.start_both()[0])
                self.assertTrue(dashboard.start_both()[0])
                spawn.assert_not_called()
            request = json.loads((project / "data/gazebo/demo-start.json").read_text())
            self.assertEqual(len(stream.getvalue().splitlines()), 1)
            self.assertAlmostEqual(float(stream.getvalue().split()[1]), request["start_at"], places=5)
            self.assertEqual(request["token"], dashboard._launch_token)
            self.assertGreater(request["start_at"], time.time())

    def test_stale_ready_state_cannot_trigger_takeoff(self):
        dashboard = Dashboard(Path("unused"), "127.0.0.1", 0, Path("unused"))
        dashboard._combined_status = "ready"
        dashboard._lite_process = SimpleNamespace(poll=lambda: None)
        for name in ("px4", "px4lite"):
            dashboard._states[name] = {"ready_for_takeoff": True, "armed": False}
            dashboard._state_seen[name] = time.monotonic() - 5
        self.assertFalse(dashboard.snapshot()["combined_start"]["can_takeoff"])
        self.assertFalse(dashboard.start_both()[0])


class LiteStandbyTest(unittest.TestCase):
    binary = Path(__file__).resolve().parents[2] / "build/simulator/px4lite_flight_demo.exe"

    def test_standby_does_not_arm_or_advance_and_trigger_preserves_determinism(self):
        baseline = subprocess.run([str(self.binary), "--fast"], capture_output=True, text=True, check=True).stdout
        process = subprocess.Popen([str(self.binary), "--fast", "--wait-for-start"],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            for _ in range(3):
                state = json.loads(process.stdout.readline())
                self.assertTrue(state["ready_for_takeoff"])
                self.assertFalse(state["armed"])
                self.assertEqual(state["sim_time_s"], 0)
                self.assertEqual(state["position_ned_m"]["z"], 0)
            deadline = time.time() + 0.3
            output, _ = process.communicate(f"START {deadline}\n", timeout=5)
            self.assertGreaterEqual(time.time(), deadline)
            flight = "".join(line + "\n" for line in output.splitlines() if not json.loads(line)["ready_for_takeoff"])
            self.assertEqual(process.returncode, 0)
            self.assertEqual(flight, baseline)
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()

    def test_closed_stdin_does_not_takeoff(self):
        result = subprocess.run([str(self.binary), "--fast", "--wait-for-start"], input="",
                                capture_output=True, text=True, timeout=5, check=True)
        self.assertTrue(all(not json.loads(line)["armed"] for line in result.stdout.splitlines()))


if __name__ == "__main__":
    unittest.main()
