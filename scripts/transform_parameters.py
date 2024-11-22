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

"""
A utility script used to transform parameters defined in the datasheet to ROS conventions.
"""

from argparse import ArgumentParser, Namespace
from dataclasses import dataclass

import numpy as np
import yaml


@dataclass
class InertialProperties:
    mass: float
    com: np.ndarray
    I_com: np.ndarray
    I_link: np.ndarray


@dataclass
class BuoyancyProperties:
    volume: float
    cob: np.ndarray


@dataclass
class DHParameters:
    d: float
    theta: float
    a: float
    alpha: float


@dataclass
class Params:
    inertial: InertialProperties
    buoyancy: BuoyancyProperties
    dh: DHParameters


def _load_file_params(fp: str) -> list[Params]:
    params = []

    with open(fp, "r") as f:
        data = yaml.safe_load(f)

        links = data["alpha_5"]["links"]

        inertial_properties = data["alpha_5"]["inertial_properties"]
        buoyancy_properties = data["alpha_5"]["buoyancy_properties"]
        dh_parameters = data["alpha_5"]["dh_parameters"]

        for link in links:
            ip = InertialProperties(
                mass=inertial_properties[link]["mass"],
                com=np.array(inertial_properties[link]["com"]),
                I_com=np.array(inertial_properties[link]["I_com"]).reshape(3, 3),
                I_link=np.array(inertial_properties[link]["I_link"]).reshape(3, 3),
            )

            bp = BuoyancyProperties(
                volume=buoyancy_properties[link]["volume"],
                cob=np.array(buoyancy_properties[link]["cob"]),
            )

            dh = DHParameters(
                d=dh_parameters[link]["d"],
                theta=dh_parameters[link]["theta_init"]
                + dh_parameters[link]["theta_c"]
                + dh_parameters[link]["theta"],
                a=dh_parameters[link]["a"],
                alpha=dh_parameters[link]["alpha"],
            )

            params.append(Params(inertial=ip, buoyancy=bp, dh=dh))

    return params


def convert_to_ros_conventions(params: list[Params]):
    # TODO: Implement this function
    '''Note: the URDF standard expect the inertia tensor to be located at the COM and aligned with the output
    coordinate system. Additionally, the URDF standard assumes a negative product of inertia convention.
    The parameters provided in Table 2 use the SolidWorks convention and do not comply with the URDF
    standard. See the documentation at wiki.ros.org/urdf/XML/link and www.mathworks.com/help/specifycustom-
    inertia for more details.
    '''
    for link in params:
        link.inertial.I_com[0,1] *= -1
        link.inertial.I_com[1,0] *= -1
        link.inertial.I_com[0,2] *= -1
        link.inertial.I_com[2,0] *= -1
        link.inertial.I_com[1,2] *= -1
        link.inertial.I_com[2,1] *= -1

        link.inertial.I_link[0,1] *= -1
        link.inertial.I_link[1,0] *= -1
        link.inertial.I_link[0,2] *= -1
        link.inertial.I_link[2,0] *= -1
        link.inertial.I_link[1,2] *= -1
        link.inertial.I_link[2,1] *= -1
    pass


def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument(
        "--file_path",
        type=str,
        required=False,
        help="Path to the parameters file",
        default="/home/parallels/ws_reach/src/reach/scripts/alpha_5_parameters.yaml",
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    params = _load_file_params(args.file_path)
    convert_to_ros_conventions(params)
