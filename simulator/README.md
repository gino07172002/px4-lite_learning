# PX4-Lite Flight Demonstration

This module provides a deterministic C++17 quadrotor plant and a runnable 3D comparison bench for PX4-Lite and the existing PX4 SITL/Gazebo setup.

## Build and test

From the repository root in PowerShell:

```powershell
cmake -S . -B build -G Ninja -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

The plant is available as `PX4Lite::SimpleQuadSimulator`. It accepts normalized four-motor commands, advances exactly one configured `fixedDtSeconds` per `step()`, and returns local-NED position, velocity, body rates, attitude, motor outputs, time, and ground contact. It models motor lag and rigid-body motion without random noise. `IQuadSimulator` is the backend boundary for a future Gazebo adapter.

## Run the 3D comparison bench

Start the local viewer from the repository root:

```powershell
python simulator/demo/server.py
```

Open [http://127.0.0.1:8765](http://127.0.0.1:8765). There are two separate steps:

1. **① 啟動模擬環境** starts PX4 SITL, the Gazebo server in the background, its MAVLink client, and the C++ Lite process. The web page supplies the 3D view; the dashboard does not open a separate Gazebo GUI or QGC window. Both vehicles remain disarmed on the ground. Lite publishes standby telemetry without advancing simulation time; PX4 keeps its GCS heartbeat and ground setpoint stream alive.
2. Once both sides have live standby telemetry, **② 同步起飛** becomes available. It sends a common start deadline two seconds ahead to both processes. They begin their flight commands at that deadline. This coordinates wall-clock commands; the two physics engines retain independent simulation clocks.

After both flights land and disarm, they return to standby. Press **② 同步起飛** again to replay without restarting Gazebo. Before readiness, the takeoff API rejects requests without starting either flight.

The C++ demo uses the existing Lab07 Commander, Lab06 attitude/rate controllers, and supplementary position controller/allocator. It flies a 2.5 m takeoff, a 4 m square, and a landing sequence. The Lab controllers, Pub/Sub, WorkQueue, and simulator fixed-step physics are unchanged by standby handling.

For the original standalone PX4 baseline that flies automatically (independent of the two web buttons), start another PowerShell:

```powershell
cd D:\px4Test
.\px4.ps1 demo
```

That existing setup runs PX4 SITL with Gazebo Harmonic. **QGroundControl is optional:** neither the dashboard nor `px4.ps1 demo` launches QGC or requires it to be running. The Python demo sends Offboard position targets and standard GCS heartbeats through its dedicated MAVLink link, and relays telemetry to UDP port 14552 for the side-by-side viewer. The heartbeat is necessary for this SITL configuration's GCS preflight check when QGC is closed. An optional QGC instance can receive its separate PX4 link on port 14550.

For a comparison without QGC, start only the dashboard and use the two buttons in order. The dashboard sets `PX4_DEMO_WAIT_FOR_START=1` for the PX4 client and passes `--wait-for-start` to Lite, so environment startup never arms or flies either vehicle. A completed legacy PX4 demo container may be replaced during environment startup; an active flight is not interrupted.

The dashboard's HTTP service binds to loopback by default. Its UDP telemetry listener binds to all local interfaces so the Docker/WSL PX4 container can send state to the Windows host. If Windows Firewall blocks UDP 14552, allow Python on the private network or run the dashboard in WSL using the Windows-visible host address for the browser.

The environment request (`POST /api/environment/start`) returns immediately while WSL/Docker checks run in the background. Takeoff is a separate `POST /api/both/start` request. On Windows, the dashboard starts a WSL keepalive process and the Docker system service. Docker commands run directly as the Ubuntu root user without loading a login shell; this does not change your default WSL user. Allow up to 120 seconds for WSL/Docker cold startup and another 150 seconds for standby telemetry. The toolbar shows the current stage and highlights failures. Keep one dashboard per UDP port to prevent competing telemetry listeners. After updating the server, restart it and refresh the browser.

Launcher regression tests can be run with `python -m unittest simulator.tests.test_dashboard -v`.

## Architecture and practice seam

The simulator owns vehicle physics only. It does not duplicate Lab Pub/Sub or WorkQueue. The demo presently passes the existing Lab controller outputs into the four-motor simulator API directly, which keeps the Lab integration exercises visible and small.

Practice extensions are intentionally left open:

1. Connect a Lab01 motor-command topic to `IQuadSimulator::setMotorCommands()` and publish `VehicleState` back as ground truth.
2. Schedule the fixed `step()` from the Lab02 WorkQueue and compare update cadence with the simulator clock.
3. Implement a Gazebo adapter behind `IQuadSimulator`, preserving the state and motor contract while Gazebo owns the physics.
4. Replace the demo's scripted setpoints with commands from QGroundControl or a MAVLink/MAVSDK client.

PX4 SITL is an independent real-PX4 comparison process. The PX4 flight controller itself owns its attitude, rate, allocation, and actuator path; the Python demo only streams Offboard position targets and relays telemetry to this viewer.
