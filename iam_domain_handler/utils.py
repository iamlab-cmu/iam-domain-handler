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

def create_formated_skill_dict(joints, end_effector_positions, time_since_skill_started):
    skill_dict = dict(skill_description='GuideMode', skill_state_dict=dict())
    skill_dict['skill_state_dict']['q'] = np.array(joints)
    skill_dict['skill_state_dict']['O_T_EE'] = np.array(end_effector_positions)
    skill_dict['skill_state_dict']['time_since_skill_started'] = np.array(time_since_skill_started)

    # The key (0 here) usually represents the absolute time when the skill was started but
    formatted_dict = {0: skill_dict}
    return formatted_dict