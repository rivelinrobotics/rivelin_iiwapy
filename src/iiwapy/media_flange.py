from typing import Dict

import rospy
from std_msgs.msg import Int8


class Outputs:
    def __init__(self, value: int) -> None:
        self.value = value
        self.output_1 = value & 0x1 > 0
        self.output_2 = value & 0x2 > 0
        self.output_3 = value & 0x4 > 0
        self.output_4 = value & 0x8 > 0

    def as_dict(self) -> Dict[int, bool]:
        return {1: self.output_1, 2: self.output_2, 3: self.output_3, 4: self.output_4}

    def __repr__(self) -> str:
        return f"<Outputs: {self.as_dict()}>"


# pylint: disable=too-many-instance-attributes
class Inputs:
    def __init__(self, value: int) -> None:
        self.value = value
        self.input_1 = value & 0x01 > 0
        self.input_2 = value & 0x02 > 0
        self.input_3 = value & 0x04 > 0
        self.input_4 = value & 0x08 > 0
        self.input_5 = value & 0x10 > 0
        self.input_6 = value & 0x20 > 0
        self.input_7 = value & 0x40 > 0
        self.input_8 = value & 0x80 > 0

    def as_dict(self) -> Dict[int, bool]:
        return {
            1: self.input_1,
            2: self.input_2,
            3: self.input_3,
            4: self.input_4,
            5: self.input_5,
            6: self.input_6,
            7: self.input_7,
            8: self.input_8,
        }

    def __repr__(self) -> str:
        return f"<Inputs: {self.as_dict()}>"


class MediaFlange:
    def __init__(self) -> None:
        self.namespace = rospy.get_namespace()

    def get_inputs(self) -> Inputs:
        inputs = rospy.wait_for_message(f"{self.namespace}state/mediaFlangeInputs", Int8, 1).data
        return Inputs(inputs)

    def get_outputs(self) -> Outputs:
        outputs = rospy.wait_for_message(f"{self.namespace}state/mediaFlangeOutputs", Int8, 1).data
        return Outputs(outputs)

    def set_outputs(
        self,
        output_1: bool = False,
        output_2: bool = False,
        output_3: bool = False,
        output_4: bool = False,
    ) -> Outputs:
        outputs = 0

        outputs |= 0x01 if output_1 else 0
        outputs |= 0x02 if output_2 else 0
        outputs |= 0x04 if output_3 else 0
        outputs |= 0x08 if output_4 else 0

        rospy.Publisher(f"{self.namespace}command/MediaFlangeOutputs", Int8, queue_size=10).publish(
            outputs
        )

        return Outputs(outputs)
