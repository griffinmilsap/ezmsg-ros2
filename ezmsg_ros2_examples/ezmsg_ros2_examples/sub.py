import asyncio
import typing

import ezmsg.core as ez
from ezmsg.ros.node import ROSNode

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
        self.STATE.chatter = self.ros_subscriber(String, self.SETTINGS.topic, 10)

    def ros_args(self) -> typing.Optional[typing.List[str]]:
        """ If we want to pass ros command line arguments through to the rclpy context
        we need to overload this method and pass them in via settings 
        If you don't plan to use ros2 run command line arguments, feel free to omit this"""
        return self.SETTINGS.ros_args

    @ez.publisher(OUTPUT_MESSAGE)
    async def pub(self) -> typing.AsyncGenerator:
        """ Publish messages received from ROS subscription into the ezmsg graph """
        while True:
            msg = await self.STATE.chatter.get()
            str_msg = msg.data
            if isinstance(str_msg, str):
                str_msg = str_msg.replace('World', '[ezmsg]')
            yield self.OUTPUT_MESSAGE, str_msg


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
