import numpy as np

from ezmsg.util.messages.axisarray import AxisArray
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

def axisarray_to_float32multiarray(msg: AxisArray) -> Float32MultiArray:

    data = np.ascontiguousarray(msg.data)
    cur_stride = 1
    dimensions = []

    for dim_name in msg.dims[::-1]:
        numel = len(msg.ax(dim_name))
        cur_stride = cur_stride * numel
        dimensions.append(
            MultiArrayDimension(
                label = dim_name,
                size = numel,
                stride = cur_stride
            )
        )

    return Float32MultiArray(
        data = data.flatten().tolist(),
        layout = MultiArrayLayout(
            dim = [d for d in dimensions[::-1]],
            data_offset = 0
        )
    )