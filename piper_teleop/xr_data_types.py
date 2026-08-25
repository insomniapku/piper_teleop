"""
XR Data Message Definition for Mock/Test Purposes
Defines the data structure for XR controller state
"""

from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class XRControllerState:
    """State of a single XR controller"""
    # Pose: [x, y, z, qx, qy, qz, qw]
    position: np.ndarray  # [x, y, z] in meters
    quaternion: np.ndarray  # [x, y, z, w] quaternion

    # Buttons and triggers
    grip_value: float  # 0.0 to 1.0
    trigger_value: float  # 0.0 to 1.0

    # Button states (right controller: A/B, left controller: X/Y)
    button_a: bool = False
    button_b: bool = False
    button_x: bool = False
    button_y: bool = False
    menu_button: bool = False
    axis_click: bool = False

    # Joystick
    joystick_x: float = 0.0  # -1.0 to 1.0
    joystick_y: float = 0.0  # -1.0 to 1.0

    # Status
    tracking_valid: bool = True
    timestamp_ns: int = 0  # nanoseconds


@dataclass
class XRHeadsetState:
    """State of XR headset"""
    position: np.ndarray  # [x, y, z] in meters
    quaternion: np.ndarray  # [x, y, z, w] quaternion
    tracking_valid: bool = True
    timestamp_ns: int = 0


@dataclass
class XRData:
    """Complete XR device state"""
    left_controller: Optional[XRControllerState] = None
    right_controller: Optional[XRControllerState] = None
    headset: Optional[XRHeadsetState] = None
    timestamp_ns: int = 0
