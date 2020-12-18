from enum import Enum
from typing import List

import rospy
from iiwa_msgs.msg import (
    CartesianImpedanceControlMode,
    CartesianQuantity,
    ControlMode,
    DesiredForceControlMode,
)
from iiwa_msgs.srv import ConfigureControlMode, ConfigureControlModeRequest

from .util import ControllerError, clamp


class DOF(Enum):
    X = 1
    Y = 2
    Z = 3
    A = 4
    B = 5
    C = 6
    ROT = 7
    TRANSL = 8
    ALL = 9


class Stiffness:
    # pylint: disable=too-many-arguments
    def __init__(
        self,
        x: float = 2000.0,
        y: float = 2000.0,
        z: float = 2000.0,
        a: float = 200.0,
        b: float = 200.0,
        c: float = 200.0,
    ) -> None:
        """
        The spring stiffness determines the extent to which the robot yields to an external force
        and deviates from its planned path.

        :param x: unit N/m must be >= 0 and <= 5000.0
        :param y: unit N/m must be >= 0 and <= 5000.0
        :param z: unit N/m must be >= 0 and <= 5000.0
        :param a: unit Nm/rad must be >= 0 and <= 300.0
        :param b: unit Nm/rad must be >= 0 and <= 300.0
        :param c: unit Nm/rad must be >= 0 and <= 300.0
        """
        self.x = clamp("x", x, 0, 5000)
        self.y = clamp("y", y, 0, 5000)
        self.z = clamp("z", z, 0, 5000)
        self.a = clamp("a", a, 0, 300)
        self.b = clamp("b", b, 0, 300)
        self.c = clamp("c", c, 0, 300)

    def to_list(self) -> List[float]:
        return [self.x, self.y, self.z, self.a, self.b, self.c]


class Damping:
    # pylint: disable=too-many-arguments
    def __init__(
        self,
        x: float = 0.7,
        y: float = 0.7,
        z: float = 0.7,
        a: float = 0.7,
        b: float = 0.7,
        c: float = 0.7,
    ) -> None:
        """
        The spring damping determines the extent to which the virtual springs oscillate after
        deflection.

        :param x: unit Lehr’s damping ratio 0.1 >= and <= 1.0
        :param y: unit Lehr’s damping ratio 0.1 >= and <= 1.0
        :param z: unit Lehr’s damping ratio 0.1 >= and <= 1.0
        :param a: unit Lehr’s damping ratio 0.1 >= and <= 1.0
        :param b: unit Lehr’s damping ratio 0.1 >= and <= 1.0
        :param c: unit Lehr’s damping ratio 0.1 >= and <= 1.0
        """
        self.x = clamp("x", x, 0.1, 1.0)
        self.y = clamp("y", y, 0.1, 1.0)
        self.z = clamp("z", z, 0.1, 1.0)
        self.a = clamp("a", a, 0.1, 1.0)
        self.b = clamp("b", b, 0.1, 1.0)
        self.c = clamp("c", c, 0.1, 1.0)

    def to_list(self) -> List[float]:
        return [self.x, self.y, self.z, self.a, self.b, self.c]


def switch_control_mode(request: ConfigureControlModeRequest) -> None:
    namespace = rospy.get_namespace()
    service = f"{namespace}configuration/ConfigureControlMode"
    rospy.wait_for_service(service)
    service_request = rospy.ServiceProxy(service, ConfigureControlMode)
    response = service_request(request)
    if not response.success:
        raise ControllerError(f"Setting control mode: {response.error}")


def position() -> None:
    switch_control_mode(ConfigureControlModeRequest(control_mode=ControlMode.POSITION_CONTROL))


def desired_force(dof: DOF, force: float, stiffness: float) -> None:
    """
    Enable desired force control mode. Raises ControllerError on failure.

    :param dof: DOF
    :param force: force in newtons (> 0 & < 10)
    :param stiffness:
    """
    if dof in (DOF.A, DOF.B, DOF.C):
        clamp("stiffness", stiffness, 0, 300.0)
    else:
        clamp("stiffness", stiffness, 0, 5000.0)

    clamp("force", force, 0, 10)  # Just limited to 10 for safety

    switch_control_mode(
        ConfigureControlModeRequest(
            control_mode=ControlMode.DESIRED_FORCE,
            desired_force=DesiredForceControlMode(
                cartesian_dof=dof.value, desired_force=force, desired_stiffness=stiffness
            ),
        )
    )


def cartesian_impedance(
    stiffness: Stiffness,
    damping: Damping,
    nullspace_stiffness: float = None,
    nullspace_damping: float = None,
) -> None:
    """
    Enable cartesian impedance control mode. Raises ControllerError on failure.

    :param stiffness: The spring stiffness determines the extent to which the robot yields to an
    external force and deviates from its planned path.
    :param damping: The spring damping determines the extent to which the virtual springs oscillate
    after deflection.
    :param nullspace_stiffness: Spring stiffness of the redundancy degree of freedom
    (type: float, unit: Nm/rad). Must be > 0 or none. The spring stiffness determines the extent to
    which the robot yields to an external force and deviates from its planned path. If not set a
    default value is used.
    :param nullspace_damping: Spring damping of the redundancy degree of freedom (type: float).
    The spring damping determines the extent to which the virtual springs oscillate after
    deflection. If not set a default value is used.
    """
    switch_control_mode(
        ConfigureControlModeRequest(
            control_mode=ControlMode.CARTESIAN_IMPEDANCE,
            cartesian_impedance=CartesianImpedanceControlMode(
                cartesian_stiffness=CartesianQuantity(*stiffness.to_list()),
                cartesian_damping=CartesianQuantity(*damping.to_list()),
                nullspace_stiffness=nullspace_stiffness,
                nullspace_damping=nullspace_damping,
            ),
        )
    )
