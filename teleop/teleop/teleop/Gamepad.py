from enum import Enum
from typing import Literal

import numpy as np


#! PS5 controller mapping
class PS5AxisMap(Enum):
    LS_H = 0  # Left stick horizontal
    LS_V = 1  # Left stick vertical
    LT = 2  # Left trigger
    RS_H = 3  # Right stick horizontal
    RS_V = 4  # Right stick vertical
    RT = 5  # Right trigger
    D_H = 6  # Dpad horizontal
    D_V = 7  # Dpad vertical


class PS5ButtonMap(Enum):
    X = 0
    C = 1  # circle
    T = 2  # triangle
    S = 3  # square
    L1 = 4
    R1 = 5
    L2 = 6
    R2 = 7
    SHARE = 8
    OPTIONS = 9
    PS = 10
    LS = 11
    RS = 12


#! Xbox controller mapping
class XboxAxisMap(Enum):
    LS_H = 0  # Left stick horizontal
    LS_V = 1  # Left stick vertical
    LT = 2  # Left trigger
    RS_H = 3  # Right stick horizontal
    RS_V = 4  # Right stick vertical
    RT = 5  # Right trigger
    D_H = 6  # Dpad horizontal
    D_V = 7  # Dpad vertical


class XboxButtonMap(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    BACK = 6
    START = 7
    GUIDE = 8  # Xbox button
    LS = 9
    RS = 10


class XboxGamepadMap:
    buttons = XboxButtonMap
    axes = XboxAxisMap


XBOX_GAMEPAD_MAP = XboxGamepadMap()


class Gamepad:
    def __init__(self):
        self.joy_axes: None | np.ndarray = None
        self.joy_buttons: None | np.ndarray = None

    def set_state(self, axes: np.ndarray, buttons: np.ndarray):
        self.joy_axes = np.array(axes)
        self.joy_buttons = np.array(buttons)

    def is_button_pressed(self, button: Enum):
        return self.joy_buttons[button.value] == 1
