import asyncio
import typing
import functools
import inspect

from dataclasses import replace

import ezmsg.core as ez
from ezmsg.core.unit import SUBSCRIBES_ATTR, task

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node, SetParametersResult
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile
    from rclpy import SignalHandlerOptions
    from rclpy.publisher import Publisher
    from rclpy.parameter import Parameter
except ImportError:
    ez.logger.error('ezmsg.ros requires rclpy from an existing ROS2 installation')
    raise

from .parameters import ROSNodeParameters

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
    _publishers: typing.Dict[str, Publisher]
    _cur_settings: ez.Settings

    async def setup(self) -> None:
        self._ctx = Context()

        rclpy.init(
            context = self._ctx, 
            # ezmsg systems are frequently terminated with sigint.
            # Its better for us to manually shut down rather than have rclpy handle that.
            signal_handler_options = SignalHandlerOptions.NO,
        )

        self._node = rclpy.create_node(self._name, context = self.context) # type: ignore
        if issubclass(self.__settings_type__, ROSNodeParameters):
            settings = self.__settings_type__.from_ros(self.node)
            self.apply_settings(settings)
            self.node.add_on_set_parameters_callback(self.on_set_parameters)

        self._loop = asyncio.get_running_loop()
        self._publishers = {}

        await super().setup()

        self._cur_settings = self.SETTINGS

    def on_set_parameters(self, params: typing.List[Parameter]) -> SetParametersResult:
        """ Callback called when params changed.  
        Return True if parameters applied successfully """
        replace_kwargs = {param.name: param.value for param in params}
        try:
            self._cur_settings = replace(self._cur_settings, **replace_kwargs)
            result = self.parameters_changed(self._cur_settings)
            return SetParametersResult(successful = True if result is None else result)
        except TypeError:
            return SetParametersResult(successful = False)
        
    def parameters_changed(self, settings: ez.Settings) -> bool:
        """ Called when parameters/settings changed from ROS """
        return True

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


V = typing.TypeVar('V', bound = ROSNode)

def ros_subscriber(msg_type: typing.Type[T], topic: str, qos_profile: typing.Union[QoSProfile, int]):
    def _subscription(func: typing.Callable[[V, T,], typing.Any]) :
        """ This decorator can be used to subscribe to a ros topic """

        @functools.wraps(func)
        async def handle_subscription(self, msg: typing.Optional[typing.Any] = None) -> typing.Optional[typing.AsyncGenerator]:

            if hasattr(func, SUBSCRIBES_ATTR):
                raise ValueError('ros_subscriber cannot wrap an ezmsg subscriber')
            
            queue = self.ros_subscriber(msg_type, topic, qos_profile)
            while True:
                from_ros = await queue.get()
                gen = func(self, from_ros)
                if isinstance(gen, typing.AsyncGenerator):
                    async for output in gen:
                        yield output
        
        return task(handle_subscription)
    return _subscription

def ros_publisher(msg_type: typing.Type[T], topic: str, qos_profile: typing.Union[QoSProfile, int]):
    def _publication(func: typing.Callable[..., typing.AsyncGenerator]):
        """ This decorator can be used to publish to a ros topic 
            NOTE: This decorator relies on a backend change to ezmsg and doesn't function in current ezmsg 3.3.4
        """

        def get_publisher(self: ROSNode) -> Publisher:
            publisher_name = f'{func.__name__}:{topic}'
            if publisher_name not in self._publishers:
                self._publishers[publisher_name] = self.node.create_publisher(msg_type, topic, qos_profile)
            return self._publishers[publisher_name]

        signature = inspect.signature(func)
        if len(signature.parameters) == 1:
            # function being wrapped is not a subscriber, we just need to call it
            @functools.wraps(func)
            async def pub_no_sub(self: ROSNode) -> typing.AsyncGenerator:
                async for stream, output in func(self):
                    if stream == topic:
                        get_publisher(self).publish(output)
                    else:
                        yield stream, output
            return task(pub_no_sub)

        else:
            # function being wrapped is a subscriber already; we need to pass message into it
            @functools.wraps(func)
            async def pub_sub(self: ROSNode, msg: typing.Optional[typing.Any] = None) -> typing.AsyncGenerator:
                async for stream, output in func(self, msg):
                    if stream == topic:
                        get_publisher(self).publish(output)
                    else:
                        yield stream, output
            return task(pub_sub)
        
    return _publication