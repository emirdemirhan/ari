"""Every scenario and vehicle file loads and runs; runs are deterministic for a seed."""

from pathlib import Path

import numpy as np
import pytest

from arisim import config
from arisim.links import SilLink
from arisim.simulator import Simulator

SIM = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("path", sorted((SIM / "vehicles").glob("*.toml")), ids=lambda p: p.stem)
def test_vehicle_files_load(path):
    vp = config.load_vehicle(path)
    assert config.derived_vehicle_values(vp)["thrust_to_weight"] > 1.2


@pytest.mark.parametrize("path", sorted((SIM / "scenarios").glob("*.toml")), ids=lambda p: p.stem)
def test_scenario_runs(path):
    sc = config.load_scenario(path)
    sc.duration_s = min(sc.duration_s, 1.0)
    vp = config.load_vehicle(SIM / "vehicles" / f"{sc.vehicle}.toml", sc.vehicle_overrides)
    res = Simulator(vp, sc, SilLink()).run()
    assert len(res.log["t"]) > 0


def test_same_seed_same_result():
    sc = config.load_scenario(SIM / "scenarios" / "hover.toml")
    sc.duration_s = 1.0
    vp = config.load_vehicle(SIM / "vehicles" / "x3_500g.toml")
    a = Simulator(vp, sc, SilLink()).run()
    b = Simulator(vp, sc, SilLink()).run()
    assert np.array_equal(a.log["z"], b.log["z"]) and np.array_equal(a.log["m1"], b.log["m1"])


def test_live_viewer_renders_headless(tmp_path, monkeypatch):
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.setenv("SDL_AUDIODRIVER", "dummy")
    from arisim.joystick import JoystickRc
    from arisim.viewer import LiveViewer

    sc = config.load_scenario(SIM / "scenarios" / "free_flight.toml")
    sc.duration_s = 0.5
    vp = config.load_vehicle(SIM / "vehicles" / "x3_500g.toml")
    rc = JoystickRc()
    rc.keys.add("up")                       # keyboard: climb
    viewer = LiveViewer(joystick=rc)
    viewer.period = 0.0
    res = Simulator(vp, sc, SilLink(), rc_source=rc, observer=viewer).run()
    viewer.screenshot(tmp_path / "frame.png")
    assert (tmp_path / "frame.png").stat().st_size > 10000
    assert res.log["ref_alt"][-1] > 0.2      # the key raised the altitude reference
