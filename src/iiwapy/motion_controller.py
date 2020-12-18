from typing import List, Optional

import actionlib
import rospy
from actionlib import GoalStatus
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import (
    CartesianPose,
    JointPosition,
    JointQuantity,
    MoveToCartesianPoseAction,
    MoveToCartesianPoseGoal,
    MoveToJointPositionAction,
    MoveToJointPositionGoal,
    RedundancyInformation,
)
from iiwa_msgs.srv import SetPTPCartesianSpeedLimits, SetPTPCartesianSpeedLimitsRequest

from .util import ControllerError


def set_speed_limits(velocity: float, acceleration: float) -> None:
    namespace = rospy.get_namespace()
    service = f"{namespace}configuration/setPTPCartesianLimits"
    rospy.wait_for_service(service)
    service_request = rospy.ServiceProxy(service, SetPTPCartesianSpeedLimits)
    response = service_request(
        SetPTPCartesianSpeedLimitsRequest(
            maxCartesianVelocity=velocity,  # m/s default = 1
            maxOrientationVelocity=0.5,  # rad/2
            maxCartesianAcceleration=acceleration,  # m/s ^ 2 default = 0.2
            maxOrientationAcceleration=0.1,  # rad/s ^ 2
            maxCartesianJerk=-1.0,  # m/s^3
            maxOrientationJerk=-1.0,  # rad/s^3
        )
    )
    if not response.success:
        raise ControllerError(f"Setting speed limit: {response.error}")


class MotionPromise:

    _failed_states = (
        GoalStatus.ABORTED,
        GoalStatus.LOST,
        GoalStatus.REJECTED,
        GoalStatus.RECALLED,
        GoalStatus.PREEMPTED,
    )

    _active_states = (
        GoalStatus.ACTIVE,
        GoalStatus.PENDING,
        GoalStatus.PREEMPTING,
        GoalStatus.RECALLING,
    )

    active: bool = True
    succeeded: bool = False
    failed: bool = False
    status: Optional[str]
    error_message: Optional[str]

    def __init__(self, client: actionlib.SimpleActionClient):
        self._client = client

    def is_active(self) -> bool:
        if not self.active:
            return False

        # Update the status check if still active
        _state = self._client.get_state()
        self.active = _state in self._active_states
        self.status = self._client.get_goal_status_text()
        self.succeeded = _state == GoalStatus.SUCCEEDED
        self.failed = _state in self._failed_states
        self.error_message = getattr(self._client.get_result(), "error", None)

        return self.active

    def wait_for_result(self, duration: int = 30) -> None:
        if not self._client.wait_for_result(rospy.Duration(secs=duration)):
            self.is_active()
            self.active = False
            raise TimeoutError(
                f"Goal timed out after {duration} seconds: {self.status} {self.error_message}"
            )

        self.is_active()

        if self.succeeded:
            return

        raise ControllerError(f"Status: {self.status} error: {self.error_message}")


class MotionController:
    start_pose: Optional[PoseStamped]
    target_pose: Optional[PoseStamped]

    def __init__(self) -> None:
        self.namespace = rospy.get_namespace()

        self.lin_motion_client = actionlib.SimpleActionClient(
            f"{self.namespace}action/move_to_cartesian_pose_lin",
            MoveToCartesianPoseAction,
        )
        self.ptp_motion_client = actionlib.SimpleActionClient(
            f"{self.namespace}action/move_to_cartesian_pose",
            MoveToCartesianPoseAction,
        )
        self.joint_motion_client = actionlib.SimpleActionClient(
            f"{self.namespace}action/move_to_joint_position",
            MoveToJointPositionAction,
        )

        self.lin_motion_client.wait_for_server()
        self.ptp_motion_client.wait_for_server()
        self.joint_motion_client.wait_for_server()

    def _reset_state(self, target: PoseStamped) -> None:
        self.start_pose = self.get_pose()
        self.target_pose = target

    def move_to_joint_position(self, axis: List[float]) -> MotionPromise:
        """
        Move to a set of joint positions

        :param axis: list of radians as floating point numbers
        :return: MotionPromise
        """
        if not len(axis) == 7:
            raise ValueError("Expecting list of 7 floating point numbers in radians ")

        goal = MoveToJointPositionGoal(joint_position=JointPosition(position=JointQuantity(*axis)))

        self.joint_motion_client.send_goal(goal)

        return MotionPromise(self.joint_motion_client)

    def move_to_pose(
        self,
        pose: PoseStamped,
        velocity: float,
        acceleration: float,
    ) -> MotionPromise:

        set_speed_limits(velocity, acceleration)

        self._reset_state(pose)

        goal = MoveToCartesianPoseGoal(
            cartesian_pose=CartesianPose(
                # -1 means the controller works out the redundancy configuration
                poseStamped=pose,
                redundancy=RedundancyInformation(e1=-1, status=-1, turn=-1),
            )
        )
        self.ptp_motion_client.send_goal(goal)

        return MotionPromise(self.ptp_motion_client)

    def move_to_pose_lin(
        self,
        pose: PoseStamped,
        velocity: float,
        acceleration: float,
    ) -> MotionPromise:

        set_speed_limits(velocity, acceleration)

        self._reset_state(pose)

        goal = MoveToCartesianPoseGoal(
            cartesian_pose=CartesianPose(
                # -1 means the controller works out the redundancy configuration
                poseStamped=pose,
                redundancy=RedundancyInformation(e1=-1, status=-1, turn=-1),
            )
        )
        self.lin_motion_client.send_goal(goal)

        return MotionPromise(self.lin_motion_client)

    def get_pose(self) -> PoseStamped:
        iiwa_pose = rospy.wait_for_message(f"{self.namespace}state/CartesianPose", CartesianPose, 1)
        return iiwa_pose.poseStamped

    def get_joint_positions(self) -> List[float]:
        p = rospy.wait_for_message(
            f"{self.namespace}state/JointPosition", JointPosition, 1
        ).position
        return [p.a1, p.a2, p.a3, p.a4, p.a5, p.a6, p.a7]
