# Copyright (c) 2024 Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), limited
# exclusively to use with products produced by Reach Robotics Pty Ltd, subject to
# the following conditions:
#
# The Software may only be used in conjunction with products manufactured or
# developed by Reach Robotics Pty Ltd.
#
# Redistributions or use of the Software in any other context, including but
# not limited to, integration, combination, or use with other products or
# software, are strictly prohibited without prior written authorization from Reach
# Robotics Pty Ltd.
#
# All copies of the Software, in whole or in part, must retain this notice and
# the above copyright notice.
#
# THIS SOFTWARE IS PROVIDED "AS IS," WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL REACH ROBOTICS
# PTY LTD BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM, OUT OF, OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os
from argparse import ArgumentParser
from collections import namedtuple
from dataclasses import dataclass
from typing import Any

import numpy as np
import yaml

DH = namedtuple("DH", ["a", "alpha", "d", "theta"])


@dataclass
class Frame:
    """Parameter information for a single frame."""

    origin: np.ndarray
    inertia: np.ndarray
    dh: DH
    mass: float
    link: str


def load_file(fp: str) -> dict[str, Any]:
    """Load a YAML file from disk.

    Args:
        fp: The file path to load.

    Returns:
        The parsed YAML data.
    """
    if not os.path.exists(fp):
        raise FileNotFoundError(f"File not found: {fp}")

    with open(fp, "r") as f:
        return yaml.safe_load(f)


def convert_inertia_convention(inertia: np.ndarray):
    """Negate the products of inertia to obtain the URDF convention.

    Args:
        inertia: Inertia tensor for a given frame.

    Returns:
        The corrected inertia tensor.
    """
    inertia_c = np.copy(inertia)  # Don't modify the matrix in place
    inertia_c[~np.eye(3, dtype=bool)] *= -1
    return inertia_c


def parse_frame(frame: dict[str, Any]) -> Frame:
    """Convert a dictionary of frame data into a Frame object.

    Args:
        frame: The parsed frame data.

    Returns:
        A Frame object with the parsed data.
    """
    origin = np.array(
        [frame["com"]["x"], frame["com"]["y"], frame["com"]["z"], 0.0, 0.0, 0.0]
    )

    inertia = np.array(
        [
            [frame["inertia"]["ixx"], frame["inertia"]["ixy"], frame["inertia"]["ixz"]],
            [frame["inertia"]["ixy"], frame["inertia"]["iyy"], frame["inertia"]["iyz"]],
            [frame["inertia"]["ixz"], frame["inertia"]["iyz"], frame["inertia"]["izz"]],
        ]
    )

    dh = DH(
        a=frame["dh"]["a"],
        alpha=frame["dh"]["alpha"],
        d=frame["dh"]["d"],
        theta=frame["dh"]["theta"],
    )

    # Convert the inertia from the Solidworks convention to the URDF convention
    inertia = convert_inertia_convention(inertia)

    return Frame(origin, inertia, dh, frame["mass"], frame["link"])


def rot_com(): ...


def rot_inertia(): ...


def parse_frames(data: str) -> list[Frame]:
    """Parse a YAML file containing frame data.

    Args:
        data: The frame data to convert.

    Returns:
        A list of Frame objects.
    """
    return [parse_frame(frame) for frame in data["frames"]]


def parse_args():
    parser = ArgumentParser()

    parser.add_argument("params", type=str, help="The file containing the frame data.")
    parser.add_argument("template", type=str, help="The template Xacro file to use.")
    parser.add_argument("output", type=str, help="The output file to write to.")

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    data = load_file(args.params)
    frames = parse_frames(data)
    print(frames)
