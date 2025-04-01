#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 Johns Hopkins University Applied Physics Lab
#
# Based on topic_tools transform.py by Daisuke Nishimatsu and enriquefernandez

"""
Usage summary.

@author: Griffin Milsap
Allows to publish messages from an ezmsg stream or one of it fields and output it on a ROS2 topic
after performing a valid python operation.
The operations are done on the message, which is taken in the variable 'm'.
* Examples (note that numpy is imported by default):
$ ros2 run topic_tools transform /imu --field orientation.x /x_str std_msgs/String 'std_msgs.msg.String(data=str(m))' --import std_msgs # noqa: E501
$ ros2 run topic_tools transform /imu --field orientation.x /x_in_degrees std_msgs/Float64 'std_msgs.msg.Float64(data=-numpy.rad2deg(m))' --import std_msgs numpy # noqa: E501
$ ros2 run topic_tools transform /imu --field orientation /norm std_msgs/Float64 'std_msgs.msg.Float64(data=numpy.sqrt(numpy.sum(numpy.array([m.x, m.y, m.z, m.w]))))' --import std_msgs numpy # noqa: E501
$ ros2 run topic_tools transform /imu --field orientation /norm std_msgs/Float64 'std_msgs.msg.Float64(data=numpy.linalg.norm([m.x, m.y, m.z, m.w]))' --import std_msgs numpy # noqa: E501
"""

import argparse
import asyncio
import importlib
import os
import sys
import typing
import dataclasses

import rclpy
import rclpy.publisher
from rclpy.utilities import remove_ros_args
from ros2topic.api import get_msg_class

from rosidl_runtime_py.utilities import get_message

import ezmsg.core as ez
from ezmsg.ros.node import ROSNode

from .qos import infer_qos_from_input_topic, QoSParameters

class ToROSSettings(ez.Settings):
    output_topic: str
    output_type: str
    expression: str
    field: typing.Optional[str]
    imports: typing.List[str] = dataclasses.field(default_factory = lambda: ['numpy'])
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

        self.STATE.expression = prepare_expression(self.SETTINGS.expression, self.SETTINGS.imports)

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
    expression: str
    field: typing.Optional[str]
    imports: typing.List[str] = dataclasses.field(default_factory = lambda: ['numpy'])
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

        self.STATE.expression = prepare_expression(self.SETTINGS.expression, self.SETTINGS.imports)

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


def prepare_expression(expression: str, imports: typing.List[str]) -> typing.Callable:
    modules = {}
    for module in imports:
        try:
            mod = importlib.import_module(module)
        except ImportError:
            ez.logger.error(f'Failed to import module: {module}')
        else:
            modules[module] = mod

    try:
        return eval(f'lambda m: {expression}', modules)
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


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description='Apply a Python operation to a topic.\n\n'
                    'A node is created that subscribes to a topic,\n'
                    'applies a Python expression to the topic (or topic\n'
                    'field) message \"m\", and publishes the result\n'
                    'through another topic.\n\n'
                    'Usage:\n\tros2 run topic_tools transform '
                    '<input topic> <output topic> <output type> '
                    '[<expression on m>] [--import numpy tf] [--field <topic_field>]\n\n'
                    'Example:\n\tros2 run topic_tools transform /imu --field orientation '
                    '/norm std_msgs/Float64'
                    '\"std_msgs.msg.Float64(data=sqrt(sum(array([m.x, m.y, m.z, m.w]))))\"'
                    ' --import std_msgs')
    parser.add_argument('input', help='Input topic or topic field.')
    parser.add_argument('output_topic', help='Output topic.')
    parser.add_argument('output_type', help='Output topic type.')
    parser.add_argument(
        'expression', default='m',
        help='Python expression to apply on the input message \"m\".'
    )

    parser.add_argument(
        '-i', '--import', dest='modules', nargs='+', default=['numpy'],
        help='List of Python modules to import.'
    )

    parser.add_argument(
        '--wait-for-start', action='store_true',
        help='Wait for input messages.'
    )

    parser.add_argument(
        '--field', type=str, default=None,
        help='Transform a selected field of a message. '
             "Use '.' to select sub-fields. "
             'For example, to transform the orientation x field of a sensor_msgs/msg/Imu message: '
             "'ros2 run topic_tools transform /imu --field orientatin.x'",
    )

    QoSParameters.setup_argparser(parser)

    args = parser.parse_args(remove_ros_args(args=argv))
    qos_parameters = QoSParameters.from_args(args)
    
    rclpy.init(args=argv)

    node = Transform(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('transform stopped cleanly')
    except BaseException:
        print('exception in transform:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()