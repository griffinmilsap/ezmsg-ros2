import typing

import ezmsg.core as ez
from ezmsg.ros.node import ROSNode, ros_subscriber

from std_msgs.msg import String


class SimpleSubscriber(ROSNode):

    OUTPUT_MESSAGE = ez.OutputStream(str)

    @ez.publisher(OUTPUT_MESSAGE)
    @ros_subscriber(String, 'chatter', 10)
    async def sub_with_decorator(self, msg: String) -> typing.AsyncGenerator:
        yield self.OUTPUT_MESSAGE, msg.data


def main():

    from ezmsg.util.debuglog import DebugLog

    sub = SimpleSubscriber()
    log = DebugLog()

    ez.run(
        SUB = sub,
        LOG = log,

        connections = (
            (sub.OUTPUT_MESSAGE, log.INPUT),
        ),

        # Need true parallelism?  Specify which units you
        # want to live in their own processes here
        process_components = (sub, )
    )

if __name__ == '__main__':
    main()
