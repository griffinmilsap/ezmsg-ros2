import ezmsg.core as ez

try:
    import rclpy
except ImportError:
    ez.logger.error('ezmsg.ros requires rclpy from an existing ROS2 installation')
    # raise

class ROSUnit(ez.Unit):
    ...