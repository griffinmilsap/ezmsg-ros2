import asyncio
import typing

import ezmsg.core as ez
from ezmsg.ros.node import ROSNode, ros_subscriber

from std_msgs.msg import String

class SimpleSubscriberSettings(ez.Settings):
    topic: str
    ros_args: typing.Optional[typing.List[str]] = None

class SimpleSubscriberState(ez.State):
    chatter: asyncio.Queue[String]

class SimpleSubscriber(ROSNode):
    SETTINGS: SimpleSubscriberSettings
    STATE: SimpleSubscriberState

    OUTPUT_MESSAGE = ez.OutputStream(str)

    async def initialize(self) -> None:
        # This method lets you dynamically choose your topic based on the SETTINGS
        self.STATE.chatter = self.ros_subscriber(String, self.SETTINGS.topic, 10)

    def ros_args(self) -> typing.Optional[typing.List[str]]:
        # If we want to pass ros command line arguments through to the rclpy context
        # we need to overload this method and pass them in via settings 
        # If you don't plan to use ros2 run command line arguments, feel free to omit this
        # these command line arguments are useful for re-defining topics and namespaces
        # within ros using the command line, but can also be done using ezmsg's
        # settings mechanism via argparse anyway.
        return self.SETTINGS.ros_args

    # Subscribe to a ros topic with a decorator
    # You can't propogate topic name from settings if you use this method
    @ros_subscriber(String, 'chatter', 10)
    @ez.publisher(OUTPUT_MESSAGE)
    async def sub_with_decorator(self, msg: String) -> typing.AsyncGenerator:
        yield self.OUTPUT_MESSAGE, msg.data

    # ... Or you can subscribe using a queue.  This lets you choose
    # your topic using self.SETTINGS.  You don't need to do both
    # Since this is publishing the message to to the OUTPUT_MESSAGE stream
    # in two publisher tasks you'll see messages logged twice.
    @ez.publisher(OUTPUT_MESSAGE)
    async def sub_with_queue(self) -> typing.AsyncGenerator:
        while True:
            msg = await self.STATE.chatter.get()
            yield self.OUTPUT_MESSAGE, msg.data


def main(args = None):


    from ezmsg.util.debuglog import DebugLog

    # We don't need to pass the topic in via settings
    # if we wanted to hard-code it, that's cool too
    # but this example shows how to use settings to configure Units.
    sub_settings = SimpleSubscriberSettings(
        topic = 'chatter',
        ros_args = args
    )

    sub = SimpleSubscriber(sub_settings)
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
