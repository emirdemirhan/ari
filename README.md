# ari

Source code of quadcopter UAV project ARI: flight controller firmware for an STM32F412
(libopencm3 + FreeRTOS) and `arisim`, a quadrotor simulator that flies the same control code on
the PC or on the board.

!! FreeRTOSConfig.h dosyasına dikkat task priority ile alakalı bazı değişiklikler yapıldı !!

## Status

The firmware flies in simulation only. Sensor data comes from the simulator over a serial
link, and motor commands go back to it; the IMU and radio drivers are not yet in the control
path and there is no motor output.

- Attitude estimation with an error-state Kalman filter (ESKF) that models rotor drag
- Height from a VL53L1X rangefinder near the ground and a barometer above it
- Cascaded PID control of attitude, yaw rate and altitude; X-quad mixer
- Watchdog and fault reporting over the serial link

## Build and flash

Needs `arm-none-eabi-gcc`, `make` and `dfu-util`.

```sh
git submodule update --init
make -C libopencm3 TARGETS=stm32/f4   # once
make                                  # build/ari.elf, build/ari.bin
make flash                            # board in the USB DFU bootloader: BOOT0 high, reset
```

After flashing, set BOOT0 low and reset the board.

## Simulator

`sim/` holds `arisim` (Python 3.11+, numpy, matplotlib, pyserial; pygame for manual flight,
Gazebo Harmonic for the optional 3D view). It runs the
controller compiled for the PC (SIL) or on the board over a USB-serial adapter on USART1
(PA9/PA10, 2 Mbaud), in lock step.

```sh
cd sim
python3 -m arisim run scenarios/hover.toml          # SIL flight, report in runs/
python3 -m arisim batch scenarios/*.toml            # all scenarios with pass/fail criteria
python3 -m arisim fly                               # fly it yourself (gamepad or keyboard)
python3 -m arisim linktest --port /dev/ttyUSB0      # check the board link
python3 -m arisim fly --gazebo --port /dev/ttyUSB0  # fly the real board, 3D view in Gazebo
```

See [sim/README.md](sim/README.md) for the models and options, and [CLAUDE.md](CLAUDE.md) for
the full technical reference: hardware map, wire protocol, control loop, conventions and known
issues.

## To Do

- Implement wind compensation with cascade pid. (Needs a ground-referenced sensor such as
  optical flow first: the IMU cannot see drift with the wind.)
- Altitude control sometimes oscillates, check that.
- Automatic pid tuning.
