# ARI — nano quadcopter flight controller (STM32F412)

Bare-metal flight controller firmware for an STM32F412RET6, built on libopencm3 + FreeRTOS,
plus a lock-step quadrotor simulator (`sim/`, package `arisim`). The simulator runs the control
code either compiled for the PC (SIL) or on the board over USART1 (PIL).

The project started as a helicopter controller and was converted to a quadcopter. Leftover
helicopter names (`main_motor_pwm`, `tail_motor_pwm`, `roll_servo_cmd`) are legacy, not design
intent.

**Current state:** the firmware only flies in simulation (SIL or PIL). The control loop takes
its sensor data from the simulator over USART1, not from the BMI160. The IMU and radio drivers
exist but are not wired into the control path. There is no motor output (PWM/DShot) yet and no
arming.

## Layout

All paths are relative to the repository root.

```
Makefile                Single hand-written Makefile
stm32f412.ld            Linker script: 512K flash @ 0x08000000, 256K RAM @ 0x20000000
STM32F412.svd           Register description for debugger peripheral views
config/FreeRTOSConfig.h
src/
  main.c                setup_device() -> HIL USART init -> create tasks -> IWDG -> start scheduler
  start.c               setup_device(): clocks, peripherals, nRF24 init, BMI160 init
  clock_config.c        HSE 8 MHz -> PLL -> 100 MHz SYSCLK; peripheral clock enables
  peripherals.c         GPIO, SPI1 (nRF24), SPI2 (BMI160), DWT delay, TIM3
  ari_timer.c           TIM3 500 Hz update interrupt -> notifies the control task
  ari_tasks.c           All FreeRTOS tasks: blink + watchdog, comm (unused), control (HIL I/O around controller_step)
  fault.c               Hard fault, stack overflow, assert and malloc-failure reports on USART1
  drivers/              bmi160.c, nrf24.c, delay.c (DWT cycle-counter busy-wait)
  communication/        communication.c (nRF24 -> cmd queue), pubsub.c (shared RC struct)
  simulation/           hil_protocol.c: frame encode/decode + CRC (pure C, shared with SIL)
                        hil_bridge.c: USART1 transport (RX interrupt + ring buffer), reply cache, reset frame
  flightcontrol/        controller.c: one 2 ms control step (estimators, PID cascade, mixer);
                          no hardware calls, also compiled into the SIL library
                        stest_wrapper.cpp: Kalman + ESKF instances, barometer fusion, tuning values, C API
include/                Mirrors src/. Much of the logic is header-only:
  flightcontrol/pid.h       calculate_pid() (static inline)
  flightcontrol/kalman.h    Kalman<StateDim, InputDim, MeasDim> template (linear)
  flightcontrol/eskf.h      AttitudeESKF: error-state filter, attitude + gyro bias + air velocity (8 error states)
  ari_math/ari_matrix.h     ari_matrix<Rows, Cols> fixed-size float matrix template
  drivers/*_wrapper.h       static inline SPI/GPIO glue + pin definitions per driver
sil/sil_lib.c           SIL entry point: bytes in -> controller_step -> bytes out (make sil)
sim/                    Python simulator (see sim/README.md): arisim/, vehicles/, scenarios/, tests/, gazebo/
tests/                  Host-only tests, not part of the firmware build
  estimator_compare.cpp     Kalman vs ESKF on a simulated quadrotor with rotor drag (make host-test)
freertos/               Vendored FreeRTOS kernel V11.1.0 (ARM_CM4F port, heap_4)
libopencm3/             Git submodule
```

## Build, flash, debug

Toolchain: `arm-none-eabi-gcc` / `g++` (13.2 is used). Run from the repository root:

```sh
git submodule update --init          # first checkout only
make -C libopencm3 TARGETS=stm32/f4  # first time only; produces libopencm3/lib/libopencm3_stm32f4.a
make                                 # -> build/ari.elf, build/ari.bin, plus a .lst per source
make clean
make flash                           # dfu-util to 0x08000000 (board must be in the USB DFU bootloader)
make host-test                       # native g++: run both estimators against a simulated truth
make sil                             # build/sil/libari_sil.so for the simulator (it runs this itself)
```

Flashing: set BOOT0 high and reset (the board appears as `0483:df11 STM32 BOOTLOADER`),
`make flash`, then BOOT0 low and reset. If BOOT0 is still high at the next reset, the board
stays in the ROM bootloader and the firmware never runs (LED dark).

Simulator (from `sim/`; details in `sim/README.md`):

```sh
python3 -m pytest tests                          # 46 tests: physics, sensors, protocol, PIL path over a pty
python3 -m arisim run scenarios/hover.toml       # one SIL run -> runs/<time>_hover_sil/{log.csv,summary.json,report.png}
python3 -m arisim batch scenarios/*.toml -v      # all scenarios; exit code 1 if any fails
python3 -m arisim run scenarios/hover.toml --port /dev/ttyUSB0   # PIL on the board
python3 -m arisim linktest --port /dev/ttyUSB0   # pre-flight check: boot message, exchanges, round trip, CPU time
python3 -m arisim fakeboard                      # virtual board on a pty, for testing PIL tooling
python3 -m arisim fly                            # manual flight, gamepad or keyboard, live view
python3 -m arisim fly --gazebo --port /dev/ttyUSB0   # PIL, flown by hand, shown in Gazebo 3D
python3 -m arisim run <scenario> --replay-rc runs/<run>/log.csv   # replay a flight's pilot inputs
```

`--gazebo` uses Gazebo only as a 3D window: `sim/gazebo/arisim_world.sdf` holds a display-only
quad (no gravity, no collisions), and `sim/gazebo/gz_relay.py` sets its pose from UDP packets
sent by `sim/arisim/gazebo_view.py`. The relay runs under the Python the gz bindings were built
for (`python3.12` here). `gazebo_view.py` removes `LIBGL_ALWAYS_SOFTWARE` from Gazebo's
environment (with it set, the GUI segfaults in Mesa's EGL when Ogre2 probes an NVIDIA GPU) and
writes Gazebo's output to `<out>/gazebo.log`. The world's `<gui>` block shows only the 3D view
(MinimalScene, GzSceneManager, InteractiveViewControl, CameraTracking); CameraTracking provides
the `/gui/follow` service the relay uses. Never move the physics into Gazebo: lock-step with the
board and the sensor models depend on `arisim` owning the clock.

- Header dependencies are tracked with `-MMD -MP` (`build/*.d`). Most of the control logic lives
  in headers (`pid.h`, `kalman.h`, `eskf.h`, `ari_matrix.h`), and editing one rebuilds its users.
  Changing compiler flags in the Makefile still needs `make clean`.
- To check a change without disturbing `build/`: `make BUILD_DIR=/some/other/dir`.
- Link diagnostics: `make BUILD_DIR=build/diag HIL_RX_DIAG=1` makes `LED_Task` print
  `RX bytes=.. fe=.. ne=.. ore=.. drop=.. crc=.. frames=..` once a second (USART1 framing /
  noise / overrun errors, ring-buffer overflows, CRC failures, good frames).
  `HIL_BAUD=115200` changes the line speed; pass the same value to the simulator with `--baud`.
- Objects are flattened into `build/` by basename, so two sources with the same filename in
  different directories collide. FreeRTOS already owns `tasks`, `queue`, `list`, `timers`, `port`.
- New `.c` files under `src/`, `src/drivers/`, `src/flightcontrol/`, `src/communication/` and
  `src/simulation/` are picked up by wildcard; `.cpp` under `src/`, `src/flightcontrol/` and
  `src/ari_math/`. A new subdirectory needs a wildcard line and an `-I` entry.
- Flags: `-O2 -g -Wall -Wextra` (switch to `-Og` temporarily for step-by-step debugging), C99,
  C++17 with `-fno-rtti -fno-exceptions`, hard-float `fpv4-sp-d16`. Linked with `g++`,
  `-nostartfiles`, `-lnosys`.
- Verification, in order of speed: `make host-test` (estimators only), `python3 -m pytest tests`
  in `sim/` (simulator models and the protocol), then scenario runs. A change to `controller.c`,
  the estimators or the PID is only checked once `arisim batch` has been run before and after it.
- `make host-test` compiles `stest_wrapper.cpp` natively and flies both filters through the
  controller's call pattern on a quadrotor truth model with rotor drag, altitude hold, sensor
  noise and a calibrated-gyro residual bias. It fails if the ESKF tilt error exceeds 0.035 rad
  or its height error 0.1 m; the fourth scenario gives the truth 30 % less drag than the ESKF
  assumes and allows 0.06 rad / 0.25 m.
- The SIL library and the firmware build the same `controller.c`, `hil_protocol.c` and
  `stest_wrapper.cpp`. Keep hardware calls (libopencm3, FreeRTOS) out of those files, or SIL
  stops building.
- The build is warning-free except for unused variables and `-Wmisleading-indentation` on the
  motor clamp lines in `controller.c`.
- The filters are global objects with no reset function. `host-test` scenarios run back to back
  on the same filter state; the simulator loads a private copy of the SIL library per run, and
  in PIL it reboots the board with a reset frame.

### Running PIL on the board

1. `make flash`. `HIL_LOCKSTEP` in `ari_tasks.c` must be 1 (the default).
2. USB-TTL adapter on USART1 (PA9 = TX -> adapter RX, PA10 = RX <- adapter TX, common GND),
   2 Mbaud 8N1, 3.3 V logic. With an FTDI adapter set its latency timer to 1 ms
   (`/sys/bus/usb-serial/devices/ttyUSB0/latency_timer`), or every step takes over 16 ms.
3. `python3 -m arisim linktest --port /dev/ttyUSB0` from `sim/`: checks the boot message
   (including the firmware's packet sizes), 2000 exchanges, round trip and controller CPU time.
4. `python3 -m arisim run <scenario> --port /dev/ttyUSB0`. The log's `exec_us` column and the
   `exec_time_max_us` metric are the controller's CPU time on the MCU.

## Hardware map

| Function | Peripheral | Pins | Notes |
|---|---|---|---|
| System clock | HSE 8 MHz -> PLL | | SYSCLK/AHB 100 MHz, APB1 50 MHz, APB2 100 MHz, flash 3 WS |
| Status LED | GPIO | PB2 | 1 Hz blink while the firmware runs; fast flashing in a fault report |
| nRF24L01+ | SPI1 (APB2) | PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CSN, PA3 CE | fPCLK/16 = 6.25 MHz, mode 0 |
| BMI160 IMU | SPI2 (APB1) | PB13 SCK, PB14 MISO, PB15 MOSI, PB12 CS | fPCLK/256 = 195 kHz (deliberately slowed, to be raised) |
| HIL link | USART1 (APB2) | PA9 TX, PA10 RX | 2,000,000 baud 8N1, RX interrupt (NVIC priority 1) into a 1 KB ring buffer, blocking TX, protocol v2 lock-step |
| Control tick | TIM3 (APB1) | | PSC 99, ARR 1999 -> 500 Hz; NVIC priority 5 |
| Watchdog | IWDG | | 500 ms, fed by `LED_Task` every 100 ms |
| Delays | DWT CYCCNT | | `delay_us()` / `delay_ms()` busy-wait, assume 100 MHz |

- TIM3's kernel clock is 100 MHz, not 50 MHz: APB1 timers run at 2x PCLK1 when the APB1
  prescaler is not 1. The comment in `ari_timer.c` says "APB1 at 100 MHz", which is wrong, but
  the PSC/ARR values are right.
- Pin definitions are duplicated: `include/peripherals.h` (`NRF_CSN`, `NRF_SPI = SPI1_BASE`, ...)
  and `include/drivers/nrf_wrapper.h` (`NRF_PIN_CSN`, `NRF_SPI = SPI1`, ...). Including both in
  one file redefines `NRF_SPI`. BMI160 pins are hard-coded in `peripherals.c` and again in
  `bmi160_wrapper.h`.
- No motor timers are configured.

## Runtime architecture

Boot: libopencm3 `reset_handler` (copies .data, zeroes .bss, enables the FPU, runs C++ static
constructors) -> `main()` -> `setup_device()` -> `start_usb_hil_system()` (prints the boot
message) -> task creation -> IWDG start -> `vTaskStartScheduler()`.

The filter objects in `stest_wrapper.cpp` are C++ globals with non-constexpr constructors.
They depend on the `.init_array` handling in `stm32f412.ld` and on the definition order inside
that one translation unit. Keep both intact.

Tasks (all created at priority 2):

| Task | Stack (words) | Started in main | Role |
|---|---|---|---|
| `LED_Task` | 128 | yes | 1 Hz blink; feeds the IWDG every 100 ms |
| `Control_Task` | 4096 | yes | Estimation, PID cascade, mixer |
| `Com_Task` | 128 | no | nRF24 link check and RC command read |

Control loop. `task_control` (`ari_tasks.c`) only does I/O:

1. With `HIL_LOCKSTEP` 1 (default): wait for the next sensor frame. The simulator is the clock;
   one frame is one 2 ms step. With 0: first wait for the TIM3 notification (500 Hz).
2. `hil_receive_full_packet()`: busy-waits on the RX ring buffer that `usart1_isr` fills,
   decodes frames, skips CRC failures, resends the cached reply for a repeated `seq`, and
   reboots the MCU on a reset frame.
3. `controller_step()` (`src/flightcontrol/controller.c`), timed with DWT `CYCCNT`.
4. `hil_send_fc_data()`: motor commands plus telemetry back to the simulator.

`controller_step()` itself:

0. Boot calibration: for the first 2 s of still IMU data (gyro < 0.15 rad/s, | |a| - g | < 0.5
   m/s^2) the motors are off and the estimators do not run; the averages become the gyro
   offset and the accelerometer level offset (`ACCEL_LEVEL_CAL` assumes the ground is level at
   power-up; a real airframe should store a level calibration instead). Telemetry flag
   `HIL_FC_FLAG_CALIBRATING` is set meanwhile. All later IMU samples are offset-corrected.
1. Predict at 500 Hz, for both estimators: angles from the gyro, altitude from the accelerometer.
2. Every 10th call (50 Hz): update both estimators (accelerometer for roll/pitch; rangefinder for
   altitude, only when the sample's VALID flag is set, through a median-of-3 outlier filter;
   barometer, see Height sources) and run the outer altitude P loop (altitude -> vertical speed
   setpoint, +2 / -1.5 m/s). The outer loop adds 0.7 x the reference's rate of change
   (feed-forward for joystick ramps).
3. Inner loops at 500 Hz: low-passed vertical speed -> collective, `HOVER_BASE` (40 %) plus
   PI (P 15, I 5) with +-30 % integrator authority, divided by cos(roll) cos(pitch). The
   integrator runs only while |vertical speed| < 0.3 m/s (hover-thrust learning; gating on the
   speed *error* deadlocked when the hover thrust was wrong). Roll/pitch angle (P 4) -> rate
   (P 4, I 6, +-10 %) -> motor delta; yaw rate (P 15, I 5, +-25 % output) -> motor delta.
   Limits: no upper altitude limit; descent speed 0.3 m/s + 1 m/s per metre of height;
   roll/pitch references limited as a vector to 30 deg total (full roll + full pitch would be
   41 deg).
   Height sources (`baro_fuse` in `stest_wrapper.cpp`): while the rangefinder has a valid
   sample it drives the height filter (precise, above the ground) and the barometer only learns
   its offset (baro - height, 2 s low-pass: ground elevation plus baro drift). Without a valid
   range (above the VL53L1X's ~4 m, blocked) the barometer minus that offset is the height
   measurement (R 0.1 m^2); the offset is frozen, so height stays relative to the ground last
   seen. The ESKF path adds the dynamic-pressure correction 0.3 (u^2 + v^2) / (2 g) from its
   air-velocity estimate (8 m/s: ~1 m); identify that coefficient on the real airframe.
   `HIL_FC_FLAG_RANGE_LOST` (0.25 s without a valid sample; back after 5 valid samples, with
   the median-filter history reset) means "height from the barometer".
   Height fail-safe (`HIL_FC_FLAG_HEIGHT_FAILSAFE`): rangefinder lost and no valid barometer
   sample for 0.5 s. The altitude estimate is not used (dead reckoning drifts ~0.3 m/s per
   second in manoeuvres): the learned hover thrust minus 3 % is held (slow descent), and tilt is
   limited to 10 deg, until either sensor is back.
   Landed state (`HIL_FC_FLAG_LANDED`): while the altitude reference is below 0.05 m and the
   measured height below 0.15 m, all motors idle at 5 % and every integrator is held at zero.
   Raising the reference takes off; lowering it to 0 in the air descends first and idles on
   touchdown. This is not arming: there is no disarm.
4. X-quad mixer with desaturation: if a motor leaves 0..100 all four are shifted together, so
   the differences (attitude control) survive and only collective changes; then clamp and fill
   `HIL_Fc_Data_t` (motors, both estimators' outputs, controller internals, flags).

PID gains and controller state are file-static in `controller.c`, set by `controller_init()`.

Two estimators run side by side every step; `USE_ESKF` at the top of `controller.c` selects which
one feeds the PIDs (1 = ESKF, the default; 0 = linear Kalman, kept for comparison).

| | Linear Kalman (`kalman_*`) | ESKF (`eskf_*`, `include/flightcontrol/eskf.h`) |
|---|---|---|
| State | roll, pitch, yaw + gyro bias (6, additive) | nominal quaternion, gyro bias, air velocity u, v; 8 error states (3 rotation, 3 bias, 2 velocity) |
| Gyro integration | body rates added to Euler angles | full quaternion kinematics |
| Accelerometer in flight | `atan2` tilt (assumes it measures gravity), trust scheduled by `estimators_set_accel_noise_scale` | rotor drag: a_x = -k u, a_y = -k v, with k = k0 sqrt(a_z / g); tilt observable through du/dt = g_b,x - k u - (w x v)_x |
| Accelerometer on the ground | same | gravity direction + zero-velocity pseudo-measurement (`eskf_set_landed`) |
| Update | 50 Hz, single sample | 50 Hz, accelerometer averaged over the 10 steps |
| Altitude | `Kalman<3,1,1>` on raw body-z accel, raw range, barometer | same model, inputs rotated to world z with the ESKF attitude; barometer with dynamic-pressure correction |

Why the ESKF: in flight a multirotor's accelerometer reads thrust plus rotor drag, not gravity,
so every gravity-based filter (the linear KF and an earlier additive-quaternion EKF) reads
acceleration as tilt and needs ad hoc accelerometer-trust schedules. The ESKF models what the
accelerometer measures. Worst tilt-estimate error in SIL against the replaced EKF:
`attitude_steps` 3.7 -> 0.4 deg, `wind_gusts` 6.0 -> about 2-4 deg (seed dependent),
`yaw_while_tilted` 4.6 -> 1.5 deg; replays of recorded PIL flights 1.1-1.6 deg (linear KF up
to 39 deg). It also estimates air velocity (`eskf_vel` in telemetry), the basis for a future
velocity/position loop and used for the barometer's dynamic-pressure correction. The
rotor-drag coefficient `ESKF_ROTOR_DRAG` (0.35 1/s, the `x3_500g` value) must be identified on
the real airframe; a +-43 % error still flies (robustness sweep) and steady-state tilt is
unaffected, but the velocity estimate scales with k_true / k_model. Worst stack frame 2.3 KB
(ground update, Joseph form).

The PID and filter code assumes exactly `Ts = 0.002` per call. The packet's `time_us` is not
used by the controller.

FreeRTOS: 1 kHz tick, preemptive, 64 KB heap_4, 5 priority levels. `configASSERT`, the stack
overflow hook (method 2) and the malloc-failed hook are on and report through `src/fault.c`.
`configMAX_SYSCALL_INTERRUPT_PRIORITY` is 5: any ISR that calls a `...FromISR` API must have
NVIC priority numerically >= 5, written as `5 << 4` for libopencm3's `nvic_set_priority`.
`usart1_isr` runs at priority 1 so FreeRTOS critical sections never delay it, and therefore must
never call FreeRTOS.

Two inter-task data paths exist and neither is used by the control task yet: a length-1 queue of
`cmd` in `communication.c` (`cmd_init` / `get_cmd` / `gnp_controlcmd`) and a critical-section
protected struct in `pubsub.c`. Pick one when wiring the radio in.

## HIL wire protocol (v2)

Defined in `include/simulation/hil_protocol.h` and mirrored in `sim/arisim/protocol.py`. Change
both together; the SIL link checks the payload sizes at load time, the PIL link checks the sizes
in the board's boot message, and the protocol tests compare a PIL run over a pty with a direct
SIL run bit for bit.

- Frame: `0xAA 0xBB | type | len | payload | crc16 LE`, CRC-16/CCITT-FALSE over type, len and
  payload. Little-endian, IEEE-754 single floats, packed structs.
- `0x01` PC -> MCU, `HIL_Sim_Data_t` (72 bytes): `seq`, `time_us`, `accel[3]` m/s^2, `gyro[3]`
  rad/s, `altitude_z` m (rangefinder), `baro_alt` m (barometric altitude, arbitrary zero),
  `rc_roll`, `rc_pitch`, `rc_yaw_rate`, `rc_thrust`, `true_roll/pitch/yaw`, `flags` (bit 0/1:
  range valid / new sample, bit 2/3: baro valid / new sample).
- `0x81` MCU -> PC, `HIL_Fc_Data_t` (92 bytes): echoed `seq`, `motor[4]` %, `kf_rpy[3]`,
  `eskf_rpy[3]`, `kf_alt`, `kf_vz`, `eskf_alt`, `eskf_vz`, `eskf_vel[2]` (air velocity, body
  x/y), `target_vz`, `total_thrust`, `roll_rate_ref`, `pitch_rate_ref`, `exec_cycles`, `flags`
  (bit 0: ESKF feeds the PIDs, bit 1: boot calibration running, bit 2: landed state, bit 3:
  rangefinder lost (height from the barometer), bit 4: height fail-safe).
- `0x02` PC -> MCU, no payload: reboot (`scb_reset_system`), so a run starts with fresh state.
- Text outside frames:
  - `Hello from STM32 (reset: <causes>) proto sim=72 fc=92` at boot. Causes come from
    `RCC_CSR` (IWDG, WWDG, POR, BOR, PIN, SFT, LPWR) and are cleared afterwards; a software
    reset also sets PIN. `linktest` and `SerialLink` refuse to run when the packet sizes differ
    from `protocol.py` (the board needs reflashing).
  - `ARI HARDFAULT pc=... lr=... cfsr=...`, `ARI STACK OVERFLOW <task>`, `ARI ASSERT
    <file>:<line>`, `ARI MALLOC FAILED` from `src/fault.c`, repeated every 0.5 s with the LED
    flashing.
- Lock-step: the PC sends one frame per step and waits for the reply with the same `seq`; on a
  timeout (10 ms) it resends the same frame, and the firmware answers from its reply cache
  without running the controller again. After 50 failed retries `SerialLink` listens and probes
  for another 1.5 s: if the board answers again the run continues and the stall is recorded in
  `link_stats.stalls`; otherwise it ends the run naming the cause (watchdog reboot, other
  reboot, fault text, or silence).

`rc_*` fields are already in firmware conventions: `rc_roll` / `rc_pitch` are -1..1 sticks
(x0.52 rad in the controller; positive pitch = nose down = forward), `rc_yaw_rate` is rad/s
(positive = counter-clockwise seen from above), and `rc_thrust` is the altitude reference in
metres, not a throttle. Stick sign conversion happens in `sim/arisim/joystick.py`.

## Conventions

- **Body frame:** x forward, y left, z up (Gazebo's FLU). The accelerometer reads +9.81 on z at
  rest. Positive roll lifts the left side, positive pitch lowers the nose, positive yaw is
  counter-clockwise seen from above.
- **Motor order** (must match `rotor.positions_m` / `rotor.spin` in `sim/vehicles/*.toml`):
  M1 front-right CCW, M2 rear-left CCW, M3 front-left CW, M4 rear-right CW.
- **Units:** SI inside the firmware (rad, rad/s, m, m/s, m/s^2). Motor commands are percent.
- **Floats:** the FPU is single precision only. Write float literals with an `f` suffix; an
  unsuffixed constant silently promotes the expression to software double arithmetic. The image
  currently links no `__aeabi_d*` routines (`arm-none-eabi-nm build/ari.elf | grep __aeabi_d`).
- **PID:** `calculate_pid` takes the derivative on the measurement, not on the error, so
  setpoint steps do not kick the D term. The state field is `y_prev`.
- **Filter tuning:** all noise values live in `stest_wrapper.cpp` (`eskf_params` for the ESKF,
  `R_BARO` and friends for the barometer). `Q` is added once per 2 ms predict step. Both
  filters start from the boot calibration, so the initial gyro-bias variance is
  (0.003 rad/s)^2. ESKF trade-off: velocity process noise `q_vel` 5e-3 lets gusts change the air
  velocity without being blamed on tilt; gyro-bias process noise `q_bias` 3e-9 keeps gusts from
  being blamed on bias. Faster bias learning (1e-7) fixes the sudden-bias-step fault but triples
  gust errors; both look like unexplained velocity changes without an absolute attitude
  reference in flight.
- **Simulator randomness:** each sensor model that is added later gets its own random stream
  (see the barometer in `simulator.py`), so adding it does not change the noise and wind of
  existing scenarios.
- **Style:** `snake_case` functions and files, enum-based error types in `ari_error.h`
  (`*_success = 0`), include guards rather than `#pragma once`, 4-space indent. Each driver puts
  its register logic in `drivers/x.c` and its pin/SPI glue in `drivers/x_wrapper.h` as static
  inline functions.
- **Language:** comments are a mix of Turkish and English. Keep whichever language the
  surrounding file uses.
- **C/C++ boundary:** C++ is used only for the matrix and filter templates. Everything the C
  code calls goes through the `extern "C"` functions in `stest_wrapper.h`.
- The misspellings `setup_perihperals` and `manuever_limits.h` are the real identifiers.

## Known issues (last reviewed 2026-10-29)

Delete an entry when it is fixed. None of the firmware has run on real flight hardware yet:
everything below comes from SIL and PIL runs with the `x3_500g` vehicle model.

`arisim batch`: 12 of 15 scenarios pass. The failures are items 1 and 3 (`sensor_faults`,
`altitude_ramps`, `yaw_while_tilted`). A robustness sweep of 150 runs (mass +-25 %, raw ESC,
slow motors, 6 ms ESC delay, 2x vibration, 40 % battery, rotor-drag 0.20 / 0.50 against the
ESKF's 0.35; 3 seeds; run before the barometer was added) had no crash, ground strike or
fly-away.

### Control and estimation

1. **Sudden gyro-bias changes are corrected slowly.** `sensor_faults`: a 3 dps step gives about
   20 deg of ESKF tilt error for several seconds; see the tuning trade-off under Conventions.
   Real bias drifts slowly with temperature; sudden faults would need fault detection (e.g.
   inflating the bias covariance on persistent innovations).
2. **Gust onsets still disturb the attitude estimate**: 2-4 deg ESKF error at the onset of a
   4 m/s gust (air velocity changes faster than modelled).
3. **`yaw_while_tilted` yaw tracking 16 dps (limit 15); `altitude_ramps` overshoots 14 cm after a
   1 m/s ramp (limit 10; feed-forward 0.7 trades lag against overshoot).**
4. **The linear Kalman filter is wrong whenever yaw and tilt combine** (53 deg in
   `yaw_while_tilted`, whose KF criterion therefore always fails); it only runs for comparison.
5. **ESKF yaw drifts (about 2 deg in 35 s in gusty wind)**: yaw is unobservable without a
   magnetometer, and gust-driven bias corrections leak into it. The controller only uses yaw
   rate.
6. **The gains are tuned only in simulation** (`x3_500g`). On the real airframe, re-check them
   with the measured vehicle file. Rate-loop P 6 / I 8 gave less motor-out tilt (4 deg) but
   more motor jitter; P 4 / I 6 was chosen.
7. **No position or velocity hold: the quad drifts with the wind.** The controller holds
   attitude (level with centred sticks) and height; nothing measures ground-relative
   horizontal motion, so a level quad moves with the air mass. The IMU cannot help: once the
   quad drifts with the air there is no force on it, and the ESKF's velocity is relative to
   the air. Holding position needs an optical-flow sensor (e.g. PMW3901 with the VL53L1X, as on
   Crazyflie-class nano drones) or GPS, a ground-velocity estimate and a velocity/position loop
   that commands roll/pitch. `free_flight_calm.toml` has no wind.
8. **Height above the rangefinder's reach is relative to the ground last seen by it** (barometer
   offset frozen): flying off a hill or over a valley above 4 m is not noticed. The barometer
   model (`sim/arisim/sensors.py`, BMP388/DPS310 class) and its firmware tuning are not checked
   against a real sensor; prop-wash pressure errors on a real airframe depend on where the
   sensor sits (shield it from the downwash, foam over the port). The height fail-safe holds the
   learned hover thrust open-loop, so it drifts slowly. `HOVER_BASE` (40 %) suits the `x3_500g`
   model; the integrator covers +-30 % from it.
9. The boot calibration assumes the vehicle is level and still for the first 2 s after reset.
   In PIL every run starts with a reset frame, so the sim always satisfies this.

### Safety

10. **No arming or disarm, no link-loss fail-safe.** The landed state idles the motors at 5 %
    (not 0) as soon as the loop runs. The HIL receive waits without a timeout: if the link
    drops, the control task waits forever and the last motor command stands.
11. **`main()` discards the result of `setup_device()`** and `xTaskCreate` return values are
    unchecked. `setup_device()` reports a BMI160 failure, but nothing acts on it. That is what
    lets HIL run on a board with no IMU fitted; real flight must refuse to start.
12. **The radio is never configured.** `nrf24_init` only sets CSN/CE and waits. `cmd_init`,
    `nrf24_config`, the address setters and `nrf24_mode_rx` are never called. Channel, address
    and payload width have to match the transmitter, which is not in this repository.
13. Payload width is ambiguous: `NRF_PAYLOAD_SIZE` (32, `nrf24.h`) is what `gnp_controlcmd`
    reads, `NRF_PAYLOADSIZE` (16, `communication.h`) is unused, and `sizeof(cmd)` is 17.
14. `task_comm` polls for commands once per second and has a 128-word stack.
15. `MAX_THROTTLE` is 3.3 and the `cmd` fields are named `*_deg` while the control loop works
    in radians; the scaling has not been reconciled with the control task.
16. The brown-out reset level is the default (~1.7 V), while 100 MHz from flash needs >= 2.7 V.
    Raise BOR to 2.7 V in the option bytes before flying on a battery.

### Sensors and peripherals

17. The BMI160 is never read in the control loop. (The boot calibration in `controller.c` works on
    whatever IMU data the controller receives, so it will apply to the BMI160 too.)
18. SPI2 at 195 kHz makes one 13-byte burst read take about 0.5 ms of busy-waiting. If it is
    raised, BMI160 needs at least 2 us of idle time between register writes.
19. `bmi160_convert_to_si` hard-codes the 2000 dps / 16 g scales regardless of the config.
20. HIL RX lessons: polling the USART from a task lost bytes whenever another task or the tick
    ran for longer than one byte time (5 us at 2 Mbaud), and on the F4 an overrun can leave
    ORE=1 with RXNE=0, after which RXNE never rises again: a loop that only waits for RXNE (as
    libopencm3's `usart_recv_blocking` does) hangs forever while the rest of the firmware,
    including the watchdog feed, keeps running. This made the board stop answering in PIL
    until 2026-10-27. `usart1_isr` now reads every byte (clearing ORE) into the ring buffer.
21. All tasks run at priority 2. Do not simply raise the control task: it busy-waits for
    received bytes, so at a higher priority it would starve everything else whenever the
    simulator is not sending. Next step: block on a task notification given from `usart1_isr`
    (which then needs priority >= 5 and the `FromISR` API).
22. TIM3 is started in `setup_perihperals()` before the tasks exist. It is safe only because of
    the NULL check in the ISR.

### Build and tooling

23. `--gc-sections` is passed without `-ffunction-sections -fdata-sections`, so it removes
    almost nothing. No map file is generated.
24. The linker script has no explicit `.ARM.exidx` placement, no stack-size reservation and no
    `end` symbol, so anything that pulls in `malloc` / `_sbrk` will fail to link.
25. `.vscode/c_cpp_properties.json` points IntelliSense at host `/usr/bin/gcc` (x64) instead of
    `arm-none-eabi-gcc`.
26. Real-time PIL depends on the USB-serial round trip: every 2 ms step waits for the board's
    reply. Measured with a PL2303 adapter at `-O2`: 1.60 ms mean round trip (0.86 ms on the
    wire, 0.17 ms controller, ~0.6 ms USB), real-time factor about 1.0; with Gazebo loading the
    PC it dropped to 0.8. The next step, if needed, is to pipeline one step (the command
    already takes effect one step late through the ESC delay); interrupt-driven RX makes that
    possible now.

## Suggested order of work

1. Measure the real airframe (mass, inertia, thrust and rpm against command, motor step
   response, IMU vibration) and put it in a `sim/vehicles/` file; the presets are the Gazebo
   X3 model.
2. Position hold: an optical-flow sensor model (PMW3901) in the simulator, a ground-velocity
   estimate and a velocity/position loop (item 7).
3. Run the scenarios in PIL to confirm SIL/PIL agreement and measure `exec_time_max_us`.
4. Add an armed/disarmed state, a link-loss timeout and motor cutoff before any real motor
   output exists; act on `setup_device()` failures (items 10, 11).
5. Separate sensor input and motor output behind an interface with a compile-time `HIL` switch,
   so the same control code runs from the simulator or from BMI160 + barometer + PWM.
6. Measure `dt` instead of assuming it, block the control task on a notification from the RX
   interrupt, then raise its priority (item 21).
7. Configure the radio (items 12 to 15) and feed RC commands to the control task.
8. Makefile: `-ffunction-sections -fdata-sections`, `-Wdouble-promotion`, a map file, and a rule
   that builds libopencm3.
