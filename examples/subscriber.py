import typing

import ezmsg.core as ez
from ezmsg.ros.unit import ROSUnit

class ROSSubscriber(ROSUnit):
    INPUT_XX = ez.InputStream(str) # TEMP
    OUTPUT_MESSAGE = ez.OutputStream(str)

    @ez.subscriber(INPUT_XX) # TEMP
    @ez.publisher(OUTPUT_MESSAGE)
    async def on_message(self, msg) -> typing.AsyncGenerator:
        yield self.OUTPUT_MESSAGE, msg

if __name__ == '__main__':
    from ezmsg.util.debuglog import DebugLog

    ros_sub = ROSSubscriber()
    log = DebugLog()

    ez.run(
        SUB = ros_sub,
        LOG = log,
        connections = (
            (ros_sub.OUTPUT_MESSAGE, log.INPUT),
        )
    )