#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 Johns Hopkins University Applied Physics Lab
#
# Inspired by topic_tools transform.py by Daisuke Nishimatsu and enriquefernandez

"""
Usage summary.

@author: Griffin Milsap
Allows to publish messages from an ezmsg stream or one of it fields and output it on a ROS2 topic
after performing a valid python operation.
The operations are done on the message, which is taken in the variable 'm'.
* Examples (note that numpy is imported by default):
$ ros2 run ezmsg_ros2 to_ros IMU --field orientation.x /imu_x 'float(m)' --import std_msgs # noqa: E501
"""

import sys
import argparse
import typing

import ezmsg.core as ez

from ezmsg.ros.relay import ToROS, ToROSSettings
from ezmsg.ros.util import QoSParameters

from rclpy.utilities import remove_ros_args

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description='Relay a stream from ezmsg to ROS\n\n'
                    'A node is created that subscribes to a stream,\n'
                    'applies a Python expression to the message (or message\n'
                    'field) \"m\", and publishes the result\n'
                    'to ROS.  Expression is typically used for message translation.\n\n'
                    'Usage:\n\tros2 run ezmsg_ros2 to_ros '
                    '<input ezmsg topic> <output ROS topic> <output ROS topic type>'
                    '[<expression on m>] [--import numpy tf] [--field <topic_field>]\n\n')
    
    parser.add_argument('input', help='Input ezmsg topic.')
    parser.add_argument('output', help='Output ROS topic.')
    parser.add_argument('output_type', help = 'Output ROS topic message type.')

    parser.add_argument(
        'expression', default='m',
        help='Python expression to apply on the input message \"m\".'
    )

    parser.add_argument(
        '-i', '--import', dest='modules', nargs='+', default=['numpy'],
        help='List of Python modules to import.'
    )

    parser.add_argument(
        '--field', type=str, default=None,
        help='Relay a selected field of a message. '
             "Use '.' to select sub-fields. "
             'For example, to relay the orientation x field of a sensor_msgs/msg/Imu message: '
             "'ros2 run ezmsg_ros2 from_ros /imu --field orientation.x'",
    )

    QoSParameters.setup_argparser(parser)
    args = parser.parse_args(remove_ros_args(args=argv))
    qos_parameters = QoSParameters.from_args(args)

    to_ros = ToROS(
        ToROSSettings(
            output_topic = args.output,
            output_type = args.output_type,
            expression = args.expression,
            field = args.field,
            modules = args.modules,
            qos_parameters = qos_parameters
        )
    )

    node_name = to_ros.node_name
    assert node_name is not None

    components: typing.Dict[str, ez.Component] = {
        node_name: to_ros
    }
    
    ez.run(
        components = components,
        connections = (
            (args.input, to_ros.INPUT),
        )
    )