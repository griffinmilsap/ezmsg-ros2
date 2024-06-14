import asyncio
import typing
import time

import numpy as np

import ezmsg.core as ez
from ezmsg.ros.node import ROSNode, ros_publisher
from ezmsg.ros.util import axisarray_to_float32multiarray
from ezmsg.util.messages.axisarray import AxisArray

from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Float32MultiArray

class PublishSignal(ROSNode):

    INPUT_SIGNAL = ez.InputStream(AxisArray)

    @ros_publisher(Float32MultiArray, 'signal', QoSPresetProfiles.SENSOR_DATA.value)
    @ez.subscriber(INPUT_SIGNAL)
    async def pub_to_ros(self, msg: AxisArray) -> typing.AsyncGenerator:
        yield 'signal', axisarray_to_float32multiarray(msg)

def main():

    class SignalGenerator(ez.Unit):
        """ A simple ezmsg unit that generates AxisArrays 
        A better example would be ezmsg.sigproc.synth.EEGSynth,
        but that would introduce a few more (un-necessary) dependencies
        """

        OUTPUT_SIGNAL = ez.OutputStream(AxisArray)

        @ez.publisher(OUTPUT_SIGNAL)
        async def generate(self) -> typing.AsyncGenerator:
            fs = 5.0 # Hz
            while True:
                now = time.time()
                yield self.OUTPUT_SIGNAL, AxisArray(
                    data = np.ones((1, 3)) * np.sin(now), 
                    dims = ['time', 'ch'],
                    axes = {
                        'time': AxisArray.Axis.TimeAxis(fs = fs, offset = now)
                    }
                )
                await asyncio.sleep(1.0 / fs)

    gen = SignalGenerator()
    pub = PublishSignal()

    ez.logger.info('Publishing AxisArrays as Float32MultiArrays to /signal')
    ez.logger.info('Try `ros2 topic echo /signal` in another terminal')

    ez.run(
        GEN = gen,
        PUB = pub,

        connections = (
            (gen.OUTPUT_SIGNAL, pub.INPUT_SIGNAL),
        ),
    )

if __name__ == '__main__':
    main()
