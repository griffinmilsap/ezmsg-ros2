import typing

from dataclasses import field

import ezmsg.core as ez
from ezmsg.ros.node import ROSNode, ros_subscriber
from ezmsg.ros.parameters import ROSNodeParameters

from rclpy.node import SetParametersResult
from rclpy.parameter import Parameter
from std_msgs.msg import String

# Derive your node SETTINGS from ROSNodeParameters to make use of ROS Parameters
class TestNodeParameters(ROSNodeParameters):
    suffix: str # a required parameter that must be defined before node can start
    int_param: int = 5 # an int parameter
    float_param: float = 2.4 # a float parameter
    bool_param: bool = False # a bool parameter
    another_param: typing.Any = None # a parameter with no type info
    int_list: list[int] = field(default_factory = list) # a parameter for a list of integers
    bytes_tuple: typing.Tuple[bytes, bytes] = field(default_factory = lambda: (b'5', b'2')) # a (clunky) parameter for a byte array
    bool_list: typing.List[bool] = field(default_factory = lambda: [False]) # a parameter for a list of bools
    float_tuple: tuple[float, float] = field(default_factory = lambda: (0.2, 0.2)) # a parameter for a tuple of floats
    string_collection: list[str] = field(default_factory = list) # a parameter for a list of strings
    _node_setting: str = 'not_a_ros_parameter' # settings defined with an underscore aren't declared or queried as ros_params

class TestNodeState(ez.State):
    cur_suffix: str

class TestNode(ROSNode):
    SETTINGS: TestNodeParameters
    STATE: TestNodeState

    OUTPUT_MESSAGE = ez.OutputStream(str)

    async def initialize(self) -> None:
        self.STATE.cur_suffix = self.SETTINGS.suffix

    def parameters_changed(self, params: typing.List[Parameter]) -> SetParametersResult:
        # We can even react to changes in parameters :D
        for param in params:
            if param.name == 'suffix' and param.type_ == Parameter.Type.STRING:
                self.STATE.cur_suffix = param.value # type: ignore
        return SetParametersResult(successful = True)

    @ros_subscriber(String, 'chatter', 10)
    @ez.publisher(OUTPUT_MESSAGE)
    async def sub_from_ros(self, msg: String) -> typing.AsyncGenerator:
        out_str = f'{msg.data} -- {self.STATE.cur_suffix}'
        yield self.OUTPUT_MESSAGE, out_str

def main():

    from ezmsg.util.debuglog import DebugLog

    node = TestNode()
    log = DebugLog()

    # Note that TestNode has never had settings applied, 
    # and the settings defines a required field with no default value.
    # This value needs to be specified within ros as a parameter

    ez.logger.info('Check parameters using `ros2 param list`')

    ez.run(
        NODE = node,
        LOG = log,

        connections = (
            (node.OUTPUT_MESSAGE, log.INPUT),
        ),
    )

    ez.logger.info('This example has a required parameter you need to set before it runs successfully')
    ez.logger.info('Try running this example with `ros2 run ezmsg_ros2_examples params --ros-args -p suffix:="ROS2 Parameters"`')


if __name__ == '__main__':
    main()
