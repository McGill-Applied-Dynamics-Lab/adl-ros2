"""ControllerSwitcherClient.on_switch: called around real switches only."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from arm_client.control.controller_switcher import ControllerSwitcherClient


def make_client(active: str, switch_ok: bool = True):
    events = []
    client = ControllerSwitcherClient(MagicMock(), on_switch=lambda: events.append("on_switch"))
    client.get_controller_list = lambda: [
        SimpleNamespace(name="joint_state_broadcaster", state="active"),
        SimpleNamespace(name="joint_trajectory_controller", state="active" if active == "joint_trajectory_controller" else "inactive"),
        SimpleNamespace(name="osc_controller", state="active" if active == "osc_controller" else "inactive"),
    ]

    def switch(to_deactivate, to_activate):
        events.append(("switch", tuple(to_deactivate), tuple(to_activate)))
        return switch_ok

    client._switch_controller = switch
    return client, events


def test_on_switch_called_before_and_after_the_switch():
    client, events = make_client(active="joint_trajectory_controller")
    assert client.switch_controller("osc_controller")
    assert events == ["on_switch", ("switch", ("joint_trajectory_controller",), ("osc_controller",)), "on_switch"]


def test_on_switch_not_called_when_already_active():
    client, events = make_client(active="osc_controller")
    assert client.switch_controller("osc_controller")
    assert events == []


def test_on_switch_not_called_after_a_failed_switch():
    client, events = make_client(active="joint_trajectory_controller", switch_ok=False)
    with pytest.raises(RuntimeError):
        client.switch_controller("osc_controller")
    assert events == ["on_switch", ("switch", ("joint_trajectory_controller",), ("osc_controller",))]


def test_no_hook_is_fine():
    client, _ = make_client(active="joint_trajectory_controller")
    client.on_switch = None
    assert client.switch_controller("osc_controller")
