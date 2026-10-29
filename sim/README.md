# arisim — lock-step quadrotor simulator for SIL and PIL

`arisim` runs the ARI flight controller against a 6-DOF quadrotor model. The same scenario
runs in two ways:

- **SIL (software in the loop):** the controller code is compiled for the PC
  (`make sil` → `build/sil/libari_sil.so`) and called directly. No hardware is needed; a 25 s
  flight takes about 4 s.
- **PIL (processor in the loop):** the firmware runs on the STM32F412 board, and sensor and
  motor frames travel over USART1. The board also reports how many CPU cycles each control
  step took.

Both modes exchange exactly the same bytes (protocol v2, `include/simulation/hil_protocol.h`).
The simulator is lock-step: simulated time advances only when the controller has answered. A
slow PC or a slow serial link therefore changes how long a run takes, never its result. A SIL
run and a PIL run with the same seed should give the same flight, apart from small
floating-point differences between x86 and the Cortex-M4.

## Quick start

```sh
cd ari/sim
python3 -m arisim info --vehicle x3_500g          # hover command, rotor speeds, thrust-to-weight
python3 -m arisim run scenarios/hover.toml        # SIL; writes runs/<time>_hover_sil/
python3 -m arisim run scenarios/hover.toml --live --realtime   # with the live view
python3 -m arisim batch scenarios/*.toml -v       # all scenarios, summary table, exit code
python3 -m arisim fly                             # manual flight: gamepad, or keyboard WASD/QE/arrows
python3 -m pytest tests                           # model, protocol and PIL-path tests
```

Requirements: Python 3.11+, numpy, matplotlib, pyserial, pygame (only for `--live` and `fly`),
pytest, and a host `gcc`/`g++` for `make sil`.

### 3D view in Gazebo, and flying it yourself

```sh
python3 -m arisim fly --gazebo                          # SIL: gamepad or keyboard, Gazebo 3D window + HUD
python3 -m arisim fly --gazebo --port /dev/ttyUSB0      # PIL: the same flight with the board in the loop
python3 -m arisim run scenarios/wind_gusts.toml --gazebo --realtime   # watch a scripted scenario
```

Gazebo is only the window. `arisim` still does the physics, the sensors and the lock-step
exchange with the controller. `--gazebo` starts `gz sim gazebo/arisim_world.sdf` (unless that
world is already running) and `gazebo/gz_relay.py`, a relay that runs under the Python that has
the gz bindings (`python3.12` here). The relay moves the quad at 60 Hz, spins the props, creates
the scenario's terrain boxes, and points the camera at the quad. The small pygame window is the
HUD: telemetry, the real-time factor, the PIL round trip, and keyboard input when there is no
gamepad (W/S pitch, A/D roll, Q/E yaw, Up/Down altitude, Esc to stop).

With the current controller the quad waits on the ground (motors idling at 5 %) until you raise
the altitude target; lowering the target to 0 lands it. The altitude target has no upper limit:
below about 4 m the rangefinder measures the height, above it the barometer does (the HUD's
RANGEFINDER LOST then just means "height from the barometer"). Only if the barometer fails too
does the controller hold hover thrust, sink slowly and limit tilt to 10 deg.
There is no position hold, so the quad drifts with the wind unless you steer against it. `--replay-rc <log.csv>` on `run`
re-flies the stick inputs of an earlier flight, for comparing controller versions on the same
flight:
`python3 -m arisim run scenarios/free_flight.toml --replay-rc runs/<flight>/log.csv`.

If the Gazebo window does not appear, read `runs/gazebo.log`. On this machine the GUI crashed
because `LIBGL_ALWAYS_SOFTWARE=1` is set; `arisim` now removes that variable for Gazebo.

In PIL each 2 ms step waits for the board's reply. If the round trip is longer than 2 ms, the
flight runs slower than real time; the HUD's real-time factor shows by how much. The fixes, in
order: set the adapter's latency timer to 1 ms (below), then use an adapter with low USB
latency.

### PIL with the board

1. Flash the firmware (`make flash` in `ari/`). `HIL_LOCKSTEP` in `src/ari_tasks.c` must be 1
   (the default).
2. Connect a USB-TTL adapter to USART1: PA9 = TX, PA10 = RX, GND. The link runs at 2 Mbaud.
3. Set the adapter's latency timer to 1 ms. FTDI chips default to 16 ms, which makes every
   lock-step round trip at least 16 ms:
   `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`.
   The simulator prints a warning if the value is higher.
4. `python3 -m arisim linktest --port /dev/ttyUSB0` checks the link before you fly: it reboots
   the board, waits for its boot message, runs 2000 lock-step exchanges, and reports the
   round trip, the controller's CPU time and the real-time factor to expect. If it fails, it
   says which wire or setting to check.
5. `python3 -m arisim run scenarios/hover.toml --port /dev/ttyUSB0`

The simulator first sends a reset frame, so the board reboots and starts with fresh estimators
(`--no-reset` skips this). If a reply is lost, the same frame is sent again. The firmware
recognises the repeated `seq` and resends its cached reply instead of running the controller a
second time.

To test the PIL tooling without the board, `python3 -m arisim fakeboard` starts a virtual
board on a pseudo-terminal: the SIL controller behind a serial port, with optional dropped
replies (`--drop 0.02`) and corrupted bytes (`--corrupt 0.001`). Run any scenario with
`--port <the printed pty>`. `tests/test_protocol.py` checks that such a run matches a direct
SIL run bit for bit.

## What is modelled

| Part | Model | Parameters (`vehicles/*.toml`) |
|---|---|---|
| Rigid body | 6-DOF, quaternion attitude, full Euler equations, RK4 at 2 kHz | `[body]` mass, inertia, drag areas |
| Rotors | thrust `k_t w^2`, yaw torque `k_q w^2`, spin-up reaction torque, gyroscopic torque, rotor drag (H-force) proportional to rotor speed x in-plane airspeed, thrust loss in climb, ground effect (Cheeseman-Bennett) | `[rotor]` |
| Motors / ESC | first-order lag, separate up/down time constants, command→rpm curve `u^e`, start threshold, command latency | `[motor]` |
| Battery | LiPo open-circuit voltage vs charge, internal resistance; current from rotor power; rotor speed scales with voltage | `[battery]` |
| Landing gear | spring-damper contact points with regularised Coulomb friction, on the ground and on terrain boxes | `[gear]` |
| IMU (BMI160) | noise density, power-on bias, bias random walk, scale and cross-axis errors, lever arm, motor vibration at rotor and blade-pass frequencies, low-pass at the datasheet bandwidth, 800 Hz ODR sample-and-hold, 16-bit quantisation, range saturation | `[imu]` |
| Rangefinder (VL53L1X) | ray cast from its mounting point along body −z, 50 Hz, latency, range limits, noise proportional to range, outliers, out-of-range behaviour | `[rangefinder]` |
| Wind | mean + Gauss-Markov turbulence + 1-cosine gusts, acting on body and rotor drag through the air-relative velocity | scenario `[wind]` |
| Terrain | ground plane plus box obstacles; they affect contact, ground effect and the rangefinder | scenario `[[terrain]]` |

Rotor drag is what makes the accelerometer meaningful in flight. Without it, as in the old
Gazebo bridge, a flying accelerometer measures only thrust along body z and reads "level"
whatever the attitude.

### Vehicle presets

- `x3_500g`: the airframe from `Simulation/Gazebosim/drone.sdf`, with thrust linear in the
  command like the old bridge, plus all the models above.
- `x3_500g_raw_esc`: rotor speed linear in the command, so thrust grows with the square of the
  command. Hover moves from about 41 % to about 64 %.
- `x3_500g_ideal`: perfect sensors, no ESC delay, no battery sag. If a scenario passes here and
  fails on `x3_500g`, the sensing is the problem, not the control.

These are placeholders until the real ARI airframe is measured. Measure the mass and
inertia (bifilar pendulum), thrust and rpm against command on a thrust stand at a known
battery voltage, the motor step response, and the IMU's vibration spectrum with the motors
running. Put the results in a new vehicle file; `base = "x3_500g.toml"` lets it override only
what differs.

## Scenarios

A scenario (`scenarios/*.toml`) holds the pilot commands, environment, faults and pass/fail
criteria. Pilot commands use the firmware's conventions: `roll`/`pitch` are sticks in −1..1
(×0.52 rad in the firmware; positive pitch is nose down, i.e. forward), `yaw_rate` is rad/s
(positive is counter-clockwise seen from above), and `altitude_m` is the altitude reference
sent as `rc_thrust`.

```toml
name = "example"
duration_s = 20
seed = 1                          # every random draw (sensor errors, noise, turbulence) follows it
vehicle = "x3_500g"

[vehicle_overrides.battery]       # optional per-scenario changes to the vehicle
initial_soc = 0.4

[[setpoints]]                     # unset channels keep their previous value
t = 2.0
altitude_m = 1.0
ramp_s = 1.0                      # optional linear ramp from the previous value

[wind]
mean_mps = [3, 0, 0]
turbulence_sigma_mps = [0.8, 0.8, 0.3]
[[wind.gusts]]
t = 10
duration_s = 2
peak_mps = [0, 4, 0]

[[terrain]]
center_m = [3, 0]
size_m = [1, 1]
height_m = 0.4

[[faults]]                        # motor_scale | gyro_bias_step (dps) | accel_bias_step (mg) | range_blocked
t = 10
type = "motor_scale"
motor = 2
value = 0.8

[[criteria]]
metric = "height_error_final_m"
max = 0.1
window = [15, 20]
```

Metrics: `no_crash` (always checked), `height_error_final_m`, `height_rmse_m`,
`height_overshoot_m`, `tilt_max_deg`, `est_tilt_error_max_deg` and `est_height_error_max_m`
(with `estimator = "kf" | "eskf" | "active"`), `attitude_tracking_rmse_deg`,
`yaw_rate_tracking_rmse_dps`, `drift_xy_max_m`, `exec_time_max_us` (PIL only),
`motor_saturation_percent`, `battery_min_v`. Heights are measured at the rangefinder, above
the terrain directly below it, because that is the quantity the altitude loop regulates.

Included scenarios: `hover`, `hover_ideal`, `altitude_steps`, `attitude_steps`,
`yaw_while_tilted`, `wind_gusts`, `motor_degraded`, `sensor_faults`, `terrain_step`,
`battery_sag`, and `free_flight` (used by `fly`).

## Outputs

Each run writes `runs/<time>_<scenario>_<sil|pil>/`:

- `log.csv`: one row per 2 ms step. It holds the truth (position, velocity, attitude, rates,
  height), references, the exact sensor values sent, motor commands and rpm, both
  estimators' outputs, controller internals, execution time, wind and battery.
- `summary.json`: the criteria results, crash info, link statistics (PIL round-trip times,
  retries), the derived vehicle numbers and the full vehicle parameters.
- `report.png`: a 12-panel plot sheet with the criteria results.

## Limits

- No magnetometer, barometer or GPS models; the firmware uses none of them yet.
- PIL exercises the control code, the HIL protocol and the CPU timing on the real MCU. It does
  not exercise the SPI drivers: sensor values arrive over USART1, not from the BMI160.
- Terrain is boxes with flat tops. The rangefinder is a single ray, without its field-of-view
  cone.
- Not modelled: blade flapping moments, rotor-to-rotor interference, ESC current limits, and
  temperature effects on the IMU.
