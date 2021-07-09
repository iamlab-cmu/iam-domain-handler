from autolab_core import RigidTransform
import numpy as np
import quaternion as qt


def EE_RigidTransform_from_state(state):
    return RigidTransform(
                translation=np.array(state['frame:franka:ee:pose/position']),
                rotation=qt.as_rotation_matrix(qt.from_float_array(state['frame:franka:ee:pose/quaternion'])),
                from_frame='franka_tool', to_frame='world'
            )


def joints_from_state(state):
    return np.array(state['frame:franka:joints/position'])
