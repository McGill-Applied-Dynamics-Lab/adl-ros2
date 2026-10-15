"""Links: simulated delays on the feedback and command links, without hardware."""

from __future__ import annotations

import numpy as np
import pytest
from fr3_haptic.links import DelayedPlant, Links, LinksConfig
from fr3_haptic.teleop import build_parser, links_config
from haptic_teleop import DelayConfig, ModelMailbox, PlantMailbox


class FakePlant:
    def __init__(self) -> None:
        self.calls: list[tuple] = []

    def aim(self, x, v):
        self.calls.append(("aim", float(x[0])))

    def command(self, x, v=None, f_ff=None):
        self.calls.append(("command", float(x[0])))

    def freeze(self):
        self.calls.append(("freeze",))


@pytest.fixture
def plant_box():
    box = PlantMailbox(1)
    yield box
    box.close()
    box.unlink()


def test_zero_delay_links_are_bypassed(plant_box):
    links = Links(LinksConfig())
    plant = FakePlant()
    assert links.plant_sink(plant_box) is plant_box
    assert links.command_plant(plant) is plant
    links.start()  # no thread without a delay
    assert links._thread is None


def test_feedback_samples_arrive_after_the_delay(plant_box):
    links = Links(LinksConfig(feedback=DelayConfig(base_ms=30)))
    sink = links.plant_sink(plant_box)
    sink.write(np.array([0.3]), np.zeros(1), np.array([2.0]), t_s=5.0, rx_s=100.0)
    links.pump(now_s=100.020)
    assert plant_box.sample is None  # still in flight
    links.pump(now_s=100.031)
    _, s = plant_box.sample
    assert (s.rx_s, s.delivered_s, s.t_s) == (100.0, 100.031, 5.0)  # measured vs delivered
    assert plant_box.sample_age_s(now_s=100.040) == pytest.approx(0.040)
    assert plant_box.delivery_age_s(now_s=100.040) == pytest.approx(0.009)
    assert links.records.feedback == [(100.0, 100.031, pytest.approx(0.030), 5.0)]


def test_commands_arrive_after_the_delay_and_freeze_drops_what_is_in_flight(monkeypatch):
    import fr3_haptic.links as links_mod

    now = [10.0]
    monkeypatch.setattr(links_mod.time, "monotonic", lambda: now[0])
    links = Links(LinksConfig(command=DelayConfig(base_ms=20)))
    plant = FakePlant()
    delayed = links.command_plant(plant)
    assert isinstance(delayed, DelayedPlant)
    delayed.aim(np.array([0.31]), np.zeros(1))
    links.pump(now_s=10.010)
    assert plant.calls == []
    links.pump(now_s=10.021)
    assert plant.calls == [("aim", 0.31)]

    now[0] = 10.030
    delayed.command(np.array([0.32]), np.zeros(1), np.zeros(1))  # in flight...
    delayed.freeze()  # ...when the robot is frozen: freeze is immediate
    links.pump(now_s=10.100)
    assert plant.calls == [("aim", 0.31), ("freeze",)]  # the late target never arrives
    delayed.aim(np.array([0.4]), np.zeros(1))  # and nothing is sent after the freeze
    links.pump(now_s=10.200)
    assert plant.calls[-1] == ("freeze",)


def test_model_link_uses_its_own_draws():
    box = ModelMailbox(7, 1)
    try:
        links = Links(LinksConfig(feedback=DelayConfig(base_ms=10, jitter_ms=5, seed=3)))
        links.plant_sink(PlantMailbox(1))
        links.model_sink(box)
        a = [links.plant_writer.line.model.sample_s() for _ in range(5)]
        b = [links.model_writer.line.model.sample_s() for _ in range(5)]
        assert a != b
    finally:
        box.close()
        box.unlink()
        links.plant_writer.mailbox.close()
        links.plant_writer.mailbox.unlink()


def test_links_config_from_flags_and_seed():
    args = build_parser().parse_args(
        ["--feedback-delay-ms", "30", "--feedback-jitter-ms", "5", "--command-delay-ms", "20", "--delay-seed", "4"]
    )
    cfg = links_config(args)
    assert (cfg.feedback.base_ms, cfg.feedback.jitter_ms, cfg.feedback.seed) == (30, 5, 4)
    assert (cfg.command.base_ms, cfg.command.jitter_ms, cfg.command.seed) == (20, 0, 5)
    unseeded = links_config(build_parser().parse_args(["--feedback-delay-ms", "10"]))
    assert isinstance(unseeded.feedback.seed, int)  # drawn, so it can be logged
    assert not links_config(build_parser().parse_args([])).enabled


def test_drain_covers_the_longest_feedback_delay():
    assert LinksConfig().drain_s() == 0.0
    assert LinksConfig(command=DelayConfig(base_ms=500)).drain_s() == 0.0  # only the feedback link replays
    assert LinksConfig(feedback=DelayConfig(base_ms=500)).drain_s() == pytest.approx(0.6)
    normal = LinksConfig(feedback=DelayConfig(base_ms=30, jitter_ms=10, jitter_dist="normal"))
    assert normal.drain_s(margin_s=0.0) == pytest.approx(0.060)


def test_worst_delivery_gap_includes_the_delay_spread():
    cfg = LinksConfig(feedback=DelayConfig(base_ms=30, jitter_ms=10, jitter_dist="uniform"))
    assert cfg.worst_delivery_gap_s(50.0) == pytest.approx(0.020 + 0.020)
    normal = LinksConfig(feedback=DelayConfig(base_ms=30, jitter_ms=10, jitter_dist="normal"))
    assert normal.worst_delivery_gap_s(100.0) == pytest.approx(0.010 + 0.060)
