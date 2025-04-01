import asyncio
import importlib
import os
import typing
import dataclasses

import rclpy
import rclpy.publisher
from ros2topic.api import get_msg_class

from rosidl_runtime_py.utilities import get_message

import ezmsg.core as ez
from ezmsg.ros.node import ROSNode

from .util import infer_qos_from_input_topic, QoSParameters

class ToROSSettings(ez.Settings):
    output_topic: str
    output_type: str
    expression: str = 'm'
    field: typing.Optional[str] = None
    modules: typing.List[str] = dataclasses.field(default_factory = lambda: ['numpy'])
    qos_parameters: QoSParameters = dataclasses.field(default_factory = QoSParameters)


class ToROSState(ez.State):
    expression: typing.Callable
    field: typing.Optional[typing.List[str]] = None
    ros_publisher: rclpy.publisher.Publisher


class ToROS(ROSNode):

    SETTINGS = ToROSSettings
    STATE = ToROSState

    INPUT = ez.InputStream(typing.Any)

    @property
    def node_name(self) -> str | None:
        return f'to_ros_{os.getpid()}'

    async def initialize(self):

        self.STATE.expression = prepare_expression(self.SETTINGS.expression, self.SETTINGS.modules)

        if self.SETTINGS.field is not None:
            self.STATE.field = list(filter(None, self.SETTINGS.field.split('.')))
            if not self.STATE.field:
                raise RuntimeError(f"Invalid field value '{self.SETTINGS.field}'")

        output_class = get_message(self.SETTINGS.output_type)
        qos_profile = self.SETTINGS.qos_parameters.profile()

        self.STATE.ros_publisher = self.node.create_publisher(output_class, self.SETTINGS.output_topic, qos_profile)

    @ez.subscriber(INPUT)
    async def from_ezmsg(self, m: typing.Any) -> None:
        try:
            res = self.STATE.expression(extract_field(m, self.STATE.field))
        except Exception:
            raise
        else:
            if not isinstance(res, (list, tuple)):
                res = [res]
            self.STATE.ros_publisher.publish(*res)
            


class FromROSSettings(ez.Settings):
    input_topic: str
    expression: str = 'm'
    field: typing.Optional[str] = None
    modules: typing.List[str] = dataclasses.field(default_factory = lambda: ['numpy'])
    wait_for_start: bool = True
    qos_parameters: QoSParameters = dataclasses.field(default_factory = QoSParameters)


class FromROSState(ez.State):
    expression: typing.Callable
    field: typing.Optional[typing.List[str]] = None
    ros_subscriber: asyncio.Queue


class FromROS(ROSNode):

    SETTINGS = FromROSSettings
    STATE = FromROSState

    OUTPUT = ez.OutputStream(typing.Any)

    @property
    def node_name(self) -> str | None:
        return f'from_ros_{os.getpid()}'

    async def initialize(self):

        self.STATE.expression = prepare_expression(self.SETTINGS.expression, self.SETTINGS.modules)

        input_class = get_msg_class(
            self.node, 
            self.SETTINGS.input_topic, 
            blocking = self.SETTINGS.wait_for_start, 
            include_hidden_topics = True
        )

        assert input_class is not None, f'Wrong input topic: {self.SETTINGS.input_topic}'

        if self.SETTINGS.field is not None:
            self.STATE.field = list(filter(None, self.SETTINGS.field.split('.')))
            if not self.STATE.field:
                raise RuntimeError(f"Invalid field value '{self.SETTINGS.field}'")

        qos_profile = (
            infer_qos_from_input_topic(self.node, self.SETTINGS.input_topic) 
            if self.SETTINGS.qos_parameters.unspecified 
            else self.SETTINGS.qos_parameters.profile()
        )

        self.STATE.ros_subscriber = self.ros_subscriber(input_class, self.SETTINGS.input_topic, qos_profile)

    @ez.publisher(OUTPUT)
    async def to_ezmsg(self) -> typing.AsyncGenerator:
        while True:
            m = await self.STATE.ros_subscriber.get()

            try:
                res = self.STATE.expression(extract_field(m, self.STATE.field))
            except Exception:
                raise
            else:
                yield self.OUTPUT, res


def prepare_expression(expression: str, modules: typing.List[str]) -> typing.Callable:
    imports = {}
    for module in modules:
        try:
            mod = importlib.import_module(module)
        except ImportError:
            ez.logger.error(f'Failed to import module: {module}')
        else:
            imports[module] = mod

    try:
        return eval(f'lambda m: {expression}', imports)
    except (NameError, UnboundLocalError) as e:
        ez.logger.error(f"Expression using variables other than 'm': {e}")
        raise
    except Exception:
        raise


def extract_field(m: typing.Any, field: typing.Optional[typing.List[str]]) -> typing.Any:
    if field is not None:
        for f in field:
            try:
                m = getattr(m, f)
            except AttributeError as ex:
                ez.logger.error(f"Invalid field '{'.'.join(field)}': {ex}")
                raise
    return m
