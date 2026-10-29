"""Command line: python3 -m arisim <command> ...   (run from ari/sim)

  run    one scenario, SIL by default or PIL with --port
  batch  several scenarios, one summary table, exit code 1 if any fails
  fly    manual flight with a joystick and the live view (real time)
  info   derived numbers for a vehicle file (hover command, rotor speeds, thrust-to-weight)
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

from . import config, report
from .gazebo_view import Observers
from .links import LinkError, SerialLink, SilLink
from .simulator import Simulator

SIM_DIR = Path(__file__).resolve().parents[1]
VEHICLE_DIR = SIM_DIR / "vehicles"


def _vehicle_path(name_or_path: str) -> Path:
    p = Path(name_or_path)
    if p.suffix == ".toml" and p.exists():
        return p
    p = VEHICLE_DIR / f"{name_or_path}.toml"
    if not p.exists():
        raise SystemExit(f"vehicle '{name_or_path}' not found (looked for {p})")
    return p


def _make_link(args):
    if args.port:
        return SerialLink(args.port, args.baud, timeout_s=args.timeout, reset=not args.no_reset)
    return SilLink(build=not args.no_build)


def run_one(scenario_path: Path, args, rc_source=None, observers=(), realtime=False) -> tuple:
    sc = config.load_scenario(scenario_path)
    if args.seed is not None:
        sc.seed = args.seed
    if args.duration is not None:
        sc.duration_s = args.duration
    vp = config.load_vehicle(_vehicle_path(args.vehicle or sc.vehicle), sc.vehicle_overrides)

    gazebo = None
    if getattr(args, "gazebo", False):
        from .gazebo_view import GazeboView
        gazebo = GazeboView(sc, gui=not args.gazebo_headless, log_dir=Path(args.out))
    observer = Observers(*observers, gazebo)

    link = None
    try:
        link = _make_link(args)
        sim = Simulator(vp, sc, link, rc_source=rc_source, realtime=realtime,
                        observer=observer if observer.observers else None)
        res = sim.run()
    finally:
        if link:
            link.close()
        observer.close()

    results = report.evaluate(res)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    out = Path(args.out) / f"{stamp}_{sc.name}_{link.name}"
    report.write_artifacts(res, results, out, plot=not args.no_plot)
    return res, results, out


def cmd_run(args):
    rc = None
    if args.replay_rc:
        from .scenario import ReplayRc
        rc = ReplayRc(args.replay_rc)
        if args.duration is None:
            args.duration = rc.duration
    res, results, out = run_one(Path(args.scenario), args, rc_source=rc, realtime=args.realtime,
                                observers=[_viewer(args)])
    passed = all(r["passed"] is not False for r in results)
    sim_t = res.log["t"][-1] if len(res.log["t"]) else 0.0
    print(f"{res.scenario.name} [{res.link}] {'PASS' if passed else 'FAIL'}"
          f"{'  CRASH: ' + res.crash_reason if res.crashed else ''}"
          f"  ({sim_t:.1f} s simulated in {res.wall_time_s:.1f} s)")
    print(report.format_results(res, results))
    print(f"  -> {out}")
    if res.link_error:
        print(f"LINK LOST at t = {res.log['t'][-1]:.2f} s: {res.link_error}", file=sys.stderr)
        return 2
    return 0 if passed else 1


def cmd_batch(args):
    rows = []
    for path in args.scenarios:
        res, results, out = run_one(Path(path), args)
        passed = all(r["passed"] is not False for r in results)
        failed = [r["metric"] + (f"[{r['estimator']}]" if r["metric"].startswith("est_") else "")
                  for r in results if r["passed"] is False]
        rows.append((res.scenario.name, passed, res.crashed, failed, out))
        print(f"{'PASS' if passed else 'FAIL'}  {res.scenario.name:24s} {res.wall_time_s:6.1f} s"
              f"{'  CRASH: ' + res.crash_reason if res.crashed else ''}"
              f"{'  failed: ' + ', '.join(failed) if failed else ''}")
        if args.verbose:
            print(report.format_results(res, results))
    n_fail = sum(1 for r in rows if not r[1])
    print(f"\n{len(rows) - n_fail}/{len(rows)} scenarios passed; artefacts in {args.out}")
    return 1 if n_fail else 0


def cmd_info(args):
    vp = config.load_vehicle(_vehicle_path(args.vehicle))
    print(f"{vp.name}: {vp.description}")
    for k, v in config.derived_vehicle_values(vp).items():
        print(f"  {k:45s} {v:10.4g}")
    return 0


def _viewer(args):
    if not getattr(args, "live", False):
        return None
    from .viewer import LiveViewer
    return LiveViewer()


def cmd_fly(args):
    from .joystick import JoystickRc
    from .viewer import LiveViewer

    rc = JoystickRc(invert_pitch=args.invert_pitch, invert_yaw=args.invert_yaw)
    viewer = LiveViewer(joystick=rc)
    args.scenario = args.scenario or str(SIM_DIR / "scenarios" / "free_flight.toml")
    res, results, out = run_one(Path(args.scenario), args, rc_source=rc, observers=[viewer], realtime=True)
    print(f"flight log -> {out}")
    if res.link_error:
        print(f"LINK LOST at t = {res.log['t'][-1]:.2f} s: {res.link_error}", file=sys.stderr)
        return 2
    return 0


def cmd_linktest(args):
    """Pre-flight check of the board link: boot message, lock-step exchanges, round trip, CPU time."""
    import statistics

    import serial

    from . import protocol
    from .protocol import SimData

    print(f"1. opening {args.port} at {args.baud} baud ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"   FAIL: {e}")
        print("   Is the adapter plugged in? Is your user in the 'dialout' group?")
        return 2
    SerialLink._warn_latency(args.port)

    print("2. sending the reset frame and waiting for 'Hello from STM32' ...")
    ser.reset_input_buffer()
    ser.write(protocol.encode_frame(protocol.TYPE_RESET, b""))
    ser.flush()
    boot = b""
    deadline = time.time() + 2.0
    while time.time() < deadline and not (b"Hello from STM32" in boot and b"\n" in boot[boot.find(b"Hello"):]):
        boot += ser.read(256)
    ser.close()
    if b"Hello from STM32" in boot:
        line = boot[boot.find(b"Hello from STM32"):].split(b"\r")[0].decode(errors="replace")
        print(f"   OK: the board rebooted and its UART TX reaches the PC: \"{line}\"")
        mismatch = protocol.check_boot_message(line)
        if mismatch:
            print(f"   FAIL: {mismatch}")
            return 1
    else:
        print(f"   FAIL: no boot message (got {len(boot)} bytes: {boot[:40]!r})")
        if not boot:
            print("   Total silence. Look at the LED on PB2 first:")
            print("     blinks at 1 Hz -> firmware runs, check the wiring below")
            print("     flashes fast   -> fault loop, but its text should arrive: check the wiring below")
            print("     dark/steady    -> firmware not running: BOOT0 still high after make flash (ROM")
            print("                       bootloader), no power, or press reset with BOOT0 low")
        print("   Check: firmware flashed (make flash), TX/RX crossed (PA9 -> adapter RX, PA10 -> adapter TX),")
        print("   common GND, adapter set to 3.3 V logic, baud rate 2000000.")
        return 1

    n = args.steps
    print(f"3. {n} lock-step exchanges with a vehicle at rest ...")
    link = SerialLink(args.port, args.baud, timeout_s=args.timeout, reset=False)
    exec_us = []
    try:
        for k in range(n):
            fc = link.step(SimData(seq=k + 1, time_us=k * 2000, accel=(0.0, 0.0, 9.81), altitude_z=0.05))
            exec_us.append(fc.exec_cycles / 100.0)
    except LinkError as e:
        print(f"   FAIL: {e}")
        print("   The board boots but does not answer frames: check PC TX -> PA10, and that the")
        print("   flashed firmware is this version (protocol v2, HIL_LOCKSTEP 1).")
        return 1
    finally:
        link.close()

    rtt = link.stats["rtt_ms"]
    mean_rtt = statistics.mean(rtt)
    p99 = sorted(rtt)[int(0.99 * (len(rtt) - 1))]
    # The PC needs about 0.35 ms per step for physics, sensors and the views
    rtf = 2.0 / (mean_rtt + 0.35)
    print(f"   OK: {n} replies, {link.stats['retries']} retries, {len(link.stats['stalls'])} stalls")
    print(f"   round trip      mean {mean_rtt:.2f} ms   p99 {p99:.2f} ms   max {max(rtt):.2f} ms")
    print(f"   controller CPU  mean {statistics.mean(exec_us):.0f} us   max {max(exec_us):.0f} us"
          f"   ({100 * max(exec_us) / 2000:.0f} % of the 2 ms budget)")
    wire_ms = (protocol.SIM_FORMAT.size + protocol.FC_FORMAT.size + 2 * 6) * 10 / args.baud * 1e3
    usb_ms = max(0.0, mean_rtt - wire_ms - statistics.mean(exec_us) / 1e3)
    driver = _usb_serial_driver(args.port)
    print(f"   round trip breakdown: wire {wire_ms:.2f} ms + controller {statistics.mean(exec_us) / 1e3:.2f} ms"
          f" + USB/driver {usb_ms:.2f} ms   (adapter driver: {driver or 'unknown'})")
    print(f"   expected real-time factor while flying: about {min(rtf, 1.0):.2f}")
    fly = "python3 -m arisim fly --gazebo --port " + args.port
    if rtf >= 1.0:
        print("   -> real time is possible. Fly with: " + fly)
    elif rtf >= 0.85:
        print(f"   -> about {100 * (1 - rtf):.0f} % slower than real time, hardly noticeable. Fly with: " + fly)
    else:
        print("   -> noticeably slower than real time (slow motion).")
    if rtf < 1.0:
        if driver == "ftdi_sio":
            print("      FTDI adapter: set its latency timer to 1 ms (see the warning above, if any).")
        if statistics.mean(exec_us) > 250:
            print("      The controller step is a large share: build the firmware with -O2 (the Makefile default).")
        print("      Beyond that, the USB adapter itself is the limit (README, PIL section).")
    return 0


def _usb_serial_driver(port: str):
    dev = os.path.basename(os.path.realpath(port))
    link = f"/sys/bus/usb-serial/devices/{dev}/driver"
    return os.path.basename(os.path.realpath(link)) if os.path.exists(link) else None


def cmd_fakeboard(args):
    from .fakeboard import FakeBoard

    board = FakeBoard(drop_reply_prob=args.drop, corrupt_prob=args.corrupt)
    print(f"fake board on {board.port}  (Ctrl+C to stop)")
    print(f"  python3 -m arisim run scenarios/hover.toml --port {board.port}")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"boots: {board.boots}, replies dropped: {board.replies_dropped}")
        board.close()
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="arisim", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        p.add_argument("--vehicle", help="vehicle name in sim/vehicles or a .toml path (default: the scenario's)")
        p.add_argument("--port", help="serial port of the board for PIL (default: SIL)")
        p.add_argument("--baud", type=int, default=2_000_000)
        p.add_argument("--timeout", type=float, default=0.01,
                       help="PIL reply timeout per attempt (s); a lost reply stalls the flight this long")
        p.add_argument("--no-reset", action="store_true", help="PIL: do not reboot the board before the run")
        p.add_argument("--no-build", action="store_true", help="SIL: do not run `make sil` first")
        p.add_argument("--seed", type=int)
        p.add_argument("--duration", type=float, help="override the scenario duration (s)")
        p.add_argument("--out", default=str(SIM_DIR / "runs"))
        p.add_argument("--no-plot", action="store_true")

    def gazebo_args(p):
        p.add_argument("--gazebo", action="store_true", help="show the flight in Gazebo (3D); arisim still does the physics")
        p.add_argument("--gazebo-headless", action="store_true", help=argparse.SUPPRESS)

    p = sub.add_parser("run", help="run one scenario")
    p.add_argument("scenario")
    p.add_argument("--live", action="store_true", help="open the live view")
    p.add_argument("--realtime", action="store_true", help="pace the simulation to wall-clock time")
    p.add_argument("--replay-rc", metavar="LOG_CSV", help="replay the pilot commands recorded in a run's log.csv")
    gazebo_args(p)
    common(p)
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("batch", help="run several scenarios")
    p.add_argument("scenarios", nargs="+")
    p.add_argument("-v", "--verbose", action="store_true")
    common(p)
    p.set_defaults(func=cmd_batch)

    p = sub.add_parser("fly", help="fly with a joystick, live view, real time")
    p.add_argument("--scenario", help="scenario for wind, terrain and faults (default: scenarios/free_flight.toml)")
    p.add_argument("--invert-pitch", action="store_true")
    p.add_argument("--invert-yaw", action="store_true")
    gazebo_args(p)
    common(p)
    p.set_defaults(func=cmd_fly)

    p = sub.add_parser("linktest", help="pre-flight check of the board link (boot, exchanges, round trip, CPU time)")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=2_000_000)
    p.add_argument("--timeout", type=float, default=0.01)
    p.add_argument("--steps", type=int, default=2000)
    p.set_defaults(func=cmd_linktest)

    p = sub.add_parser("fakeboard", help="virtual board on a pty (SIL behind a serial port) to test PIL tooling")
    p.add_argument("--drop", type=float, default=0.0, help="probability of dropping a reply")
    p.add_argument("--corrupt", type=float, default=0.0, help="probability of corrupting each received byte")
    p.set_defaults(func=cmd_fakeboard)

    p = sub.add_parser("info", help="print derived vehicle numbers")
    p.add_argument("--vehicle", default="x3_500g")
    p.set_defaults(func=cmd_info)

    args = ap.parse_args(argv)
    try:
        return args.func(args)
    except (LinkError, config.ConfigError, RuntimeError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
