import asyncio
import typing

import ezmsg.core as ez
from ezmsg.ros.node import ROSNode

from rclpy.publisher import Publisher
from std_msgs.msg import String


class SimplePublisherState(ez.State):
    ros_pub: Publisher

class SimplePublisher(ROSNode):
    STATE: SimplePublisherState

    INPUT_MESSAGE = ez.InputStream(str)

    async def initialize(self) -> None:
        self.STATE.ros_pub = self.node.create_publisher(String, 'chatter', 10)

    @ez.subscriber(INPUT_MESSAGE)
    async def pub_to_ros(self, msg: str) -> None:
        ez.logger.info(f'Publishing into ROS: {msg}')
        self.STATE.ros_pub.publish(String(data = msg)) # yup.


def main(args = None):

    class MessageGenerator(ez.Unit):
        """ A simple ezmsg unit that generates strings """

        OUTPUT_MESSAGE = ez.OutputStream(str)

        @ez.publisher(OUTPUT_MESSAGE)
        async def generate(self) -> typing.AsyncGenerator:
            n = 0
            while True:
                n += 1
                yield self.OUTPUT_MESSAGE, f'Hello from [ezmsg]: {n}'
                await asyncio.sleep(1.0)

    gen = MessageGenerator()
    pub = SimplePublisher()

    ez.run(
        GEN = gen,
        PUB = pub,

        connections = (
            (gen.OUTPUT_MESSAGE, pub.INPUT_MESSAGE),
        ),
    )

if __name__ == '__main__':
    main()
