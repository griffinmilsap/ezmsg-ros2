import asyncio
import typing

import ezmsg.core as ez

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile
    from rclpy import SignalHandlerOptions

except ImportError:
    ez.logger.error('ezmsg.ros requires rclpy from an existing ROS2 installation')


T = typing.TypeVar('T')

class ROSNode(ez.Unit):
    """ Any ROSNode will register itself with ROS as a Node via its own dedicated Context. 
    Inherit from this class to define a ROSNode for your ezmsg system.

    ```
        from std_msgs.msg import String

        class MySubscriber(ROSNode):

            OUTPUT_MESSAGE = ez.OutputStream(str)

            async def initialize(self) -> None:
                self.STATE.chatter = self.ros_subscriber(String, 'chatter', 10)

            @ez.publisher(OUTPUT_MESSAGE)
            async def pub(self) -> typing.AsyncGenerator:
                while True:
                    msg = await self.STATE.chatter.get()
                    yield self.OUTPUT_MESSAGE, str_msg
    ```
    """

    _ctx: Context = None
    _node: Node = None
    _loop: asyncio.AbstractEventLoop

    async def setup(self) -> None:
        self._ctx = Context()

        rclpy.init(
            args = self.ros_args(),
            context = self._ctx, 
            domain_id = self.domain_id(),
            # ezmsg systems are frequently terminated with sigint.
            # Its better for us to manually shut down rather than have rclpy handle that.
            signal_handler_options = SignalHandlerOptions.NO,
        )

        self._node = rclpy.create_node(self._name, context = self.context) # type: ignore
        self._loop = asyncio.get_running_loop()
        await super().setup()

    # Overload ros_args in a child class to provide specific args
    # Settings have been applied by the time this method is called
    def ros_args(self) -> typing.Optional[typing.List[str]]:
        return None
    
    # Overload domain_id in a child class to change the domain_id
    # Settings have been applied by the time this method is called
    def domain_id(self) -> int:
        return 0

    @property
    def node(self) -> Node:
        return self._node
    
    @property
    def context(self) -> Context:
        return self._ctx

    async def shutdown(self) -> None:
        if self.node: 
            self.node.destroy_node()
        if self.context: 
            rclpy.try_shutdown(context = self.context)

    def ros_subscriber(self, msg_type: typing.Type[T], topic: str, qos_profile: typing.Union[QoSProfile, int]) -> asyncio.Queue[T]:
        queue = asyncio.Queue()
        ez.logger.info(f'Subscribing to {topic} from ROS node: {self._node.get_name()}')
        self._node.create_subscription(msg_type, topic, 
            lambda msg: self._loop.call_soon_threadsafe(queue.put_nowait, msg), # type: ignore
            qos_profile = qos_profile
        )
        return queue

    @ez.task
    async def spin(self) -> None:
        executor = SingleThreadedExecutor(context = self.context)
        await self._loop.run_in_executor(None, rclpy.spin, self.node, executor)
