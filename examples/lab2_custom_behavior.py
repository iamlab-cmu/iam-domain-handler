import rospy
import json
import time
import math
import numpy as np
from autolab_core import RigidTransform
from frankapy.utils import convert_rigid_transform_to_array
import quaternion as qt

from iam_domain_handler.domain_client import DomainClient

AZURE_KINECT_INTRINSICS = '/home/student/Prog/iam-interface/camera-calibration/calib/azure_kinect.intr'
AZURE_KINECT_EXTRINSICS = '/home/student/Prog/iam-interface/camera-calibration/calib/azure_kinect_overhead/azure_kinect_overhead_to_world.tf'

if __name__ == '__main__':
    #rospy.init_node('run_domain_client')
    domain = DomainClient()

    while not rospy.is_shutdown():

        button_inputs = []
        text_inputs = []

        query_params = {
            'display_type' : 0, 
            'buttons' : [
                {
                    'name' : 'Pick Up Object',
                    'text' : '',
                }
            ]
        }

        query_response = domain.run_query_until_done('Starting Menu', query_params)
        button_inputs = query_response['button_inputs']            

        if button_inputs['Pick Up Object'] == 1:
            domain.clear_human_inputs()
            query_params = {
                'instruction_text' : 'Press Start when you are safely away from the robot.',
                'buttons' : [
                    {
                        'name' : 'Start',
                        'text' : '',
                    },
                    {
                        'name' : 'Cancel',
                        'text' : '',
                    },
                ],
            }
            query_response = domain.run_query_until_done('Pick and Place 1', query_params)
            button_inputs = query_response['button_inputs']

            if button_inputs['Start'] == 1:
                domain.clear_human_inputs()

                skill_params = {
                    'duration' : 5,
                    'dt' : 0.01,
                    'goal_joints' : [0, -math.pi / 4, 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
                }
                skill_id = domain.run_skill('one_step_joint', json.dumps(skill_params))

                domain.wait_until_skill_done(skill_id)
                intermediate_height_offset = 0.11

                azure_kinect_to_world_transform = RigidTransform.load(AZURE_KINECT_EXTRINSICS) 
                current_state = domain.state

                # TODO: FILL OUT Block Position and Goal Pose
                block_position = None
                block_pose = RigidTransform(translation=block_position, to_frame='azure_kinect_overhead', from_frame='azure_kinect_overhead')
                goal_pose = None
                goal_position = goal_pose.translation

                ##############################################

                print(goal_position)
                intermediate_pose = RigidTransform(rotation=np.array([
                    [1, 0, 0],
                    [0, -1, 0],
                    [0, 0, -1],
                ]), translation=np.array([goal_position[0], goal_position[1], goal_position[2] + intermediate_height_offset]))

                grasp_pose = RigidTransform(rotation=np.array([
                    [1, 0, 0],
                    [0, -1, 0],
                    [0, 0, -1],
                ]), translation=np.array([goal_position[0], goal_position[1], goal_position[2]]))
                # _____________
                skill_params = {
                    'duration' : 5,
                    'dt' : 0.01,
                    'goal_pose' : list(convert_rigid_transform_to_array(intermediate_pose))
                }
                skill_id = domain.run_skill('one_step_pose', json.dumps(skill_params))

                domain.wait_until_skill_done(skill_id)
                
                skill_params = {
                    'duration' : 5,
                    'dt' : 0.01,
                    'goal_pose' : list(convert_rigid_transform_to_array(grasp_pose))
                }
                skill_id = domain.run_skill('one_step_pose', json.dumps(skill_params))
                domain.wait_until_skill_done(skill_id)
                
                skill_id = domain.run_skill('close_gripper', '')
                domain.wait_until_skill_done(skill_id)

                skill_id = domain.run_skill('open_gripper', '')
                domain.wait_until_skill_done(skill_id)
            

            elif button_inputs['Cancel'] == 1:
                domain.clear_human_inputs()

        else:
            print("ERROR: Button not found")
