import asyncio
import typing

import ezmsg.core as ez
from ezmsg.ros.node import ROSNode, ros_subscriber, ros_publisher
from ezmsg.ros.parameters import ROSNodeParameters

from std_msgs.msg import String

# Derive your node SETTINGS from ROSNodeParameters to make use of ROS Parameters
class TestNodeParameters(ROSNodeParameters):
    suffix: str # a required parameter that must be defined before node can start
    int_param: int = 5 # an int parameter

    # settings defined with an underscore aren't declared or queried as ros_params
    _node_setting: str = 'not_a_ros_parameter'

class TestNodeState(ez.State):
    cur_suffix: str

class TestNode(ROSNode):
    SETTINGS: TestNodeParameters
    STATE: TestNodeState

    OUTPUT_MESSAGE = ez.OutputStream(str)

    async def initialize(self) -> None:
        # Settings from `ros2 run --ros-args -p` have already been applied here
        self.STATE.cur_suffix = self.SETTINGS.suffix

    def parameters_changed(self, settings: TestNodeParameters) -> bool:
        # Whenever parameters change in ROS, this callback is called with updated settings
        self.STATE.cur_suffix = settings.suffix
        return True

    @ros_subscriber(String, 'chatter', 10)
    @ez.publisher(OUTPUT_MESSAGE)
    async def sub_from_ros(self, msg: String) -> typing.AsyncGenerator:
        # We can subscribe to ROS topics using a decorator just like in ezmsg
        out_str = f'{msg.data} -- {self.STATE.cur_suffix}'
        yield self.OUTPUT_MESSAGE, out_str

    @ros_publisher(String, 'ezmsg_chatter', 10)
    async def pub_from_ezmsg(self) -> typing.AsyncGenerator:
        # And we can publish to ros topics using a decorator too.
        n = 0
        while True:
            await asyncio.sleep(1.0)
            n += 1
            yield 'ezmsg_chatter', String(data = f'Hello from ezmsg: {n}')

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
