"""Wire protocol: CRC, framing, C/Python agreement, lock-step duplicate handling, PIL over a pty."""

import numpy as np
import pytest

from arisim import config, protocol
from arisim.fakeboard import FakeBoard
from arisim.links import SerialLink, SilLibrary, SilLink
from arisim.protocol import Decoder, FcData, SimData
from arisim.simulator import Simulator

SIM = __import__("pathlib").Path(__file__).resolve().parents[1]


def test_crc16_ccitt_false_check_value():
    assert protocol.crc16(b"123456789") == 0x29B1


def test_decoder_resyncs_after_garbage_and_rejects_bad_crc():
    frame = SimData(seq=7).frame()
    bad = bytearray(frame)
    bad[10] ^= 0xFF
    dec = Decoder()
    frames = dec.feed(b"Hello from STM32\r\n\xaa\xaa" + bytes(bad) + frame[:5])
    frames += dec.feed(frame[5:])
    assert len(frames) == 1 and frames[0][0] == protocol.TYPE_SIM_DATA
    assert dec.crc_errors == 1


def test_sil_struct_sizes_and_roundtrip():
    link = SilLink()           # checks C sizeof() against the Python formats
    fc = link.step(SimData(seq=1, altitude_z=0.05))
    assert fc.seq == 1 and len(fc.motor) == 4
    link.close()


def test_duplicate_seq_returns_cached_reply_without_stepping():
    lib = SilLibrary()
    f1 = SimData(seq=1, altitude_z=0.05, accel=(0.0, 0.0, 9.81)).frame()
    r1 = lib.exchange(f1)
    r1_again = lib.exchange(f1)
    assert r1 == r1_again
    r2 = lib.exchange(SimData(seq=2, altitude_z=0.05, accel=(0.0, 0.0, 9.81)).frame())
    # A second real step moves the estimator, so the reply changes
    assert FcData.unpack(Decoder().feed(r2)[0][1]).seq == 2
    lib.close()


def test_pil_over_pty_with_link_faults_matches_sil():
    sc = config.load_scenario(SIM / "scenarios" / "hover.toml")
    sc.duration_s = 1.0
    vp = config.load_vehicle(SIM / "vehicles" / "x3_500g.toml")
    ref = Simulator(vp, sc, SilLink()).run()

    board = FakeBoard(drop_reply_prob=0.03, corrupt_prob=0.0005, seed=5)
    link = SerialLink(board.port, timeout_s=0.02, boot_wait_s=0.2)
    try:
        res = Simulator(vp, sc, link).run()
    finally:
        link.close()
        board.close()
    assert res.link_stats["retries"] > 0
    assert board.boots == 2                         # initial boot + reset frame
    for col in ref.log:
        if col != "exec_us":
            assert np.array_equal(ref.log[col], res.log[col]), col


def test_pil_reports_a_board_reboot_mid_run():
    sc = config.load_scenario(SIM / "scenarios" / "hover.toml")
    sc.duration_s = 3.0
    vp = config.load_vehicle(SIM / "vehicles" / "x3_500g.toml")
    board = FakeBoard(reboot_after=500, seed=1)
    link = SerialLink(board.port, timeout_s=0.02, boot_wait_s=0.2)
    try:
        res = Simulator(vp, sc, link).run()
    finally:
        link.close()
        board.close()
    assert "rebooted during the run" in res.link_error and "IWDG" in res.link_error
    assert 0.9 < res.log["t"][-1] < 1.1          # stopped at the reboot, log kept


def test_pil_link_resumes_after_stall():
    # The board stops answering for longer than all retries, then answers again: lock-step lets
    # the flight continue, and the stall is reported instead of ending the run
    sc = config.load_scenario(SIM / "scenarios" / "hover.toml")
    sc.duration_s = 2.0
    vp = config.load_vehicle(SIM / "vehicles" / "x3_500g.toml")
    board = FakeBoard(stall_after=300, stall_s=0.8, seed=1)
    link = SerialLink(board.port, timeout_s=0.01, retries=20, boot_wait_s=0.2)
    try:
        res = Simulator(vp, sc, link).run()
    finally:
        link.close()
        board.close()
    assert not res.link_error
    assert res.log["t"][-1] > 1.99
    assert len(res.link_stats["stalls"]) == 1 and "answered again" in res.link_stats["stalls"][0][1]
