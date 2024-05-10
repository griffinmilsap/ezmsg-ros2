import typing

from dataclasses import fields, MISSING

import ezmsg.core as ez

import rclpy.exceptions

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import ParameterDescriptor
except ImportError:
    ez.logger.error('ezmsg.ros requires rclpy from an existing ROS2 installation')
    raise

class ROSNodeParameters(ez.Settings):
    """ An ez.Settings subclass that makes working with ROS parameters easier """
    
    @classmethod
    def from_ros(cls, node: Node):
        parameters = []
        for field in fields(cls):
            # Fields that start with an underscore won't be 
            # declared as ROS parameters or queried as such.
            if field.name.startswith('_'): 
                continue

            value = None
            type = from_field_type(field.type)
            if field.default is not MISSING:
                value = field.default
            elif field.default_factory is not MISSING:
                value = field.default_factory()

            descriptor = ParameterDescriptor(name = field.name, type = type.value)
            parameters.append((field.name, value, descriptor))

        node.declare_parameters('', parameters)

        kwargs = {}
        for field in fields(cls):
            if field.name.startswith('_'):
                continue

            try:
                param = node.get_parameter(name = field.name)
                kwargs[field.name] = param.value
            except rclpy.exceptions.ParameterUninitializedException:
                ez.logger.info(f'{field.name} -- PARAMETER UNINITIALIZED')

        return cls(**kwargs)

def from_field_type(field_type: typing.Type):
    """
    Get a Parameter.Type from a given type.

    :return: A Parameter.Type corresponding to the instance type of the given value.
    :raises: TypeError if the conversion to a type was not possible.
    """
    if field_type == typing.Any:
        return Parameter.Type.NOT_SET
    elif field_type == bool:
        return Parameter.Type.BOOL
    elif field_type == int:
        return Parameter.Type.INTEGER
    elif field_type == float:
        return Parameter.Type.DOUBLE
    elif field_type == str:
        return Parameter.Type.STRING
    elif hasattr(field_type, '__origin__') and field_type.__origin__ in (list, tuple):
        element_types = typing.get_args(field_type)
        if all(v == bytes for v in element_types):
            return Parameter.Type.BYTE_ARRAY
        elif all(v == bool for v in element_types):
            return Parameter.Type.BOOL_ARRAY
        elif all(v == int for v in element_types):
            return Parameter.Type.INTEGER_ARRAY
        elif all(v == float for v in element_types):
            return Parameter.Type.DOUBLE_ARRAY
        elif all(v == str for v in element_types):
            return Parameter.Type.STRING_ARRAY
        else:
            raise TypeError(
                'The given value is not a list of one of the allowed types'
                f" '{field_type}'.")
    else:
        raise TypeError(
            f"The given value is not one of the allowed types '{field_type}'.")