import asyncio
import typing

import ezmsg.core as ez
from ezmsg.ros.node import ROSNode, ros_publisher

from std_msgs.msg import String

class SimplePublisher(ROSNode):

    INPUT_MESSAGE = ez.InputStream(str)

    @ros_publisher(String, 'chatter', 10)
    @ez.subscriber(INPUT_MESSAGE)
    async def pub_to_ros(self, msg: str) -> typing.AsyncGenerator:
        yield 'chatter', String(data = msg)

def main():

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
