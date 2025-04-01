#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 Johns Hopkins University Applied Physics Lab
#
# Inspired by topic_tools transform.py by Daisuke Nishimatsu and enriquefernandez

"""
Usage summary.

@author: Griffin Milsap
Allows to publish messages from a ROS topic or one of it fields and output it on an ezmsg stream
after performing a valid python operation.
The operations are done on the message, which is taken in the variable 'm'.
* Examples (note that numpy is imported by default):
$ ros2 run ezmsg_ros2 from_ros /imu --field orientation.x IMU_X 'float(m)' --import std_msgs # noqa: E501
"""

import sys
import argparse
import typing

import ezmsg.core as ez

from ezmsg.ros.relay import FromROS, FromROSSettings
from ezmsg.ros.util import QoSParameters

from rclpy.utilities import remove_ros_args

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description='Relay a topic from ROS to ezmsg\n\n'
                    'A node is created that subscribes to a topic,\n'
                    'applies a Python expression to the topic (or topic\n'
                    'field) message \"m\", and publishes the result\n'
                    'to ezmsg.  Expression is typically used for message translation.\n\n'
                    'Usage:\n\tros2 run ezmsg_ros2 from_ros '
                    '<input ROS topic> <output ezmsg topic> '
                    '[<expression on m>] [--import numpy tf] [--field <topic_field>]\n\n'
                    'Example:\n\tros2 run ezmsg_ros2 from_ros /imu --field orientation '
                    'IMU_NORM '
                    '\"numpy.sqrt(sum(numpy.array([m.x, m.y, m.z, m.w]))))\"'
                    ' --import numpy')
    
    parser.add_argument('input', help='Input ROS topic.')
    parser.add_argument('output', help='Output ezmsg topic.')

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
        help='Relay a selected field of a message. '
             "Use '.' to select sub-fields. "
             'For example, to relay the orientation x field of a sensor_msgs/msg/Imu message: '
             "'ros2 run ezmsg_ros2 from_ros /imu --field orientation.x'",
    )

    QoSParameters.setup_argparser(parser)
    args = parser.parse_args(remove_ros_args(args=argv))
    qos_parameters = QoSParameters.from_args(args)

    from_ros = FromROS(
            FromROSSettings(
            input_topic = args.input,
            expression = args.expression,
            field = args.field,
            modules = args.modules,
            wait_for_start = args.wait_for_start,
            qos_parameters = qos_parameters
        )
    )

    node_name = from_ros.node_name
    assert node_name is not None

    components: typing.Dict[str, ez.Component] = {
        node_name: from_ros
    }
    
    ez.run(
        components = components,
        connections = (
            (from_ros.OUTPUT, args.output),
        )
    )