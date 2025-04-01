import os
import rclpy
import typing

from dataclasses import dataclass, field, fields
from argparse import ArgumentParser

import ezmsg.core as ez

import rclpy.qos
from rclpy import Node
from rclpy.qos import (
    QoSDurabilityPolicy, 
    QoSPresetProfiles, 
    QoSReliabilityPolicy, 
    QoSProfile
)
from ros2topic.api import qos_profile_from_short_keys

DEFAULT_QOS = 'sensor_data'

@dataclass
class QoSParameters(typing.Protocol):
    qos_profile: typing.Optional[str] = None
    qos_reliability: typing.Optional[str] = None
    qos_durability: typing.Optional[str] = None
    qos_depth: typing.Optional[int] = None
    qos_history: typing.Optional[str] = None

    @property
    def unspecified(self) -> bool:
        return all([v is None for v in fields(self)])
    
    @classmethod
    def from_args(cls, args: "QoSParameters"):
        return cls(
            qos_profile = args.qos_profile,
            qos_reliability = args.qos_reliability,
            qos_durability = args.qos_durability,
            qos_depth = args.qos_depth,
            qos_history = args.qos_history
        )
    
    def profile(self) -> QoSProfile:
        return qos_profile_from_short_keys(
            self.qos_profile if self.qos_profile is not None else DEFAULT_QOS,
            reliability = self.qos_reliability, 
            durability = self.qos_durability,
            depth = self.qos_depth,
            history = self.qos_history
        )
        
    @staticmethod
    def setup_argparser(parser: ArgumentParser, default: str = DEFAULT_QOS) -> None:

        default_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(default)

        parser.add_argument(
            '--qos-profile',
            choices=rclpy.qos.QoSPresetProfiles.short_keys(),
            help=f'Quality of service preset profile to subscribe with (default: {default})'
        )
        
        parser.add_argument(
            '--qos-depth', metavar='N', type=int,
            help='Queue size setting to subscribe with '
                '(overrides depth value of --qos-profile option)')
        
        parser.add_argument(
            '--qos-history',
            choices=rclpy.qos.QoSHistoryPolicy.short_keys(),
            help='History of samples setting to subscribe with '
                '(overrides history value of --qos-profile option, default: {})'
                .format(default_profile.history.short_key))
        
        parser.add_argument(
            '--qos-reliability',
            choices=rclpy.qos.QoSReliabilityPolicy.short_keys(),
            help='Quality of service reliability setting to subscribe with '
                '(overrides reliability value of --qos-profile option, default: '
                'Automatically match existing publishers )')
        
        parser.add_argument(
            '--qos-durability',
            choices=rclpy.qos.QoSDurabilityPolicy.short_keys(),
            help='Quality of service durability setting to subscribe with '
                '(overrides durability value of --qos-profile option, default: '
                'Automatically match existing publishers )')


def infer_qos_from_input_topic(node: Node, topic_name: str) -> QoSProfile:

    out_profile = QoSPresetProfiles.get_from_short_key(DEFAULT_QOS)
    reliability_reliable_endpoints_count = 0
    durability_transient_local_endpoints_count = 0

    pubs_info = node.get_publishers_info_by_topic(topic_name)
    publishers_count = len(pubs_info)
    if publishers_count == 0:
        return out_profile

    for info in pubs_info:
        if (info.qos_profile.reliability == QoSReliabilityPolicy.RELIABLE):
            reliability_reliable_endpoints_count += 1
        if (info.qos_profile.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL):
            durability_transient_local_endpoints_count += 1

    # If all endpoints are reliable, ask for reliable
    if reliability_reliable_endpoints_count == publishers_count:
        out_profile.reliability = QoSReliabilityPolicy.RELIABLE
    else:
        if reliability_reliable_endpoints_count > 0:
            ez.logger.info(
                'Some, but not all, publishers are offering '
                'QoSReliabilityPolicy.RELIABLE. Falling back to '
                'QoSReliabilityPolicy.BEST_EFFORT as it will connect '
                'to all publishers'
            )
        out_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

    # If all endpoints are transient_local, ask for transient_local
    if durability_transient_local_endpoints_count == publishers_count:
        out_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    else:
        if durability_transient_local_endpoints_count > 0:
            ez.logger.info(
                'Some, but not all, publishers are offering '
                'QoSDurabilityPolicy.TRANSIENT_LOCAL. Falling back to '
                'QoSDurabilityPolicy.VOLATILE as it will connect '
                'to all publishers'
            )
        out_profile.durability = QoSDurabilityPolicy.VOLATILE

    return out_profile


