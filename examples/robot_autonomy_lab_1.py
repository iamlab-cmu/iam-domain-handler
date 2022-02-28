import rospy
import json
import time
import math
import numpy as np
from autolab_core import RigidTransform
from frankapy.utils import convert_rigid_transform_to_array
import quaternion as qt

from iam_domain_handler.domain_client import DomainClient

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
                    'name' : 'Record Trajectory',
                    'text' : '',
                },
                {
                    'name' : 'Replay Trajectory',
                    'text' : '',
                },
                {
                    'name' : 'Record Point Trajectory',
                    'text' : '',
                },
                {
                    'name' : 'Replay Point Trajectory',
                    'text' : '',
                },
                {
                    'name' : 'Train DMP',
                    'text' : '',
                },
                {
                    'name' : 'Execute DMP',
                    'text' : '',
                },
                {
                    'name' : 'Pick Up Object',
                    'text' : '',
                },
            ]
        }

        query_response = domain.run_query_until_done('Starting Menu', query_params)
        button_inputs = query_response['button_inputs']

        if button_inputs['Record Trajectory'] == 1:
            domain.clear_human_inputs()
            query_params = {
                'instruction_text' : 'Hold onto the robot and press the Start button to Move the Robot to the Starting Position.',
                'buttons' : [
                    {
                        'name' : 'Start',
                        'text' : '',
                    },
                    {
                        'name' : 'Cancel',
                        'text' : '',
                    },
                ]
            }

            query_response = domain.run_query_until_done('Teaching 1', query_params)
            button_inputs = query_response['button_inputs']

            if button_inputs['Start'] == 1:
                domain.clear_human_inputs()
                skill_params = {
                    'duration' : 10,
                    'dt' : 0.01
                }
                skill_id = domain.run_skill('zero_force', json.dumps(skill_params))

                query_params = {
                    'instruction_text' : 'Move the Robot to the Starting Position and Press Done when Completed.',
                    'buttons' : [
                        {
                            'name' : 'Done',
                            'text' : '',
                        },
                        {
                            'name' : 'Cancel',
                            'text' : '',
                        },
                    ]
                }

                query_response = domain.run_query_until_done('Teaching 2', query_params)
                button_inputs = query_response['button_inputs']
                    
                if button_inputs['Done'] == 1:
                    domain.clear_human_inputs()
                    if domain.get_skill_status(skill_id) == 'running':
                        domain.cancel_skill(skill_id)

                    query_params = {
                        'instruction_text' : 'Enter the name of the skill and its duration. Then hold onto the robot and press Start.',
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
                        'text_inputs' : [
                            {
                                'name' : 'skill_name',
                                'text' : 'Skill Name',
                                'value' : '',
                            },
                            {
                                'name' : 'skill_duration',
                                'text' : 'Skill Duration',
                                'value' : '10',
                            },
                        ]
                    }

                    query_response = domain.run_query_until_done('Teaching 3', query_params)
                    button_inputs = query_response['button_inputs']
                    text_inputs = query_response['text_inputs']

                    if button_inputs['Start'] == 1:
                        skill_name = text_inputs['skill_name']
                        skill_duration = float(text_inputs['skill_duration'])
                        domain.clear_human_inputs()

                        skill_params = {
                            'duration' : skill_duration,
                            'dt' : 0.01
                        }
                        skill_id = domain.run_skill('record_trajectory', json.dumps(skill_params))

                        query_params = {
                            'instruction_text' : 'Press Done when Completed.',
                            'buttons' : [
                                {
                                    'name' : 'Done',
                                    'text' : '',
                                },
                                {
                                    'name' : 'Cancel',
                                    'text' : '',
                                },
                            ]
                        }

                        query_response = domain.run_query_until_done('Teaching 4', query_params)
                        button_inputs = query_response['button_inputs']
            
                        if button_inputs['Done'] == 1:
                            recorded_trajectory = domain.get_memory_objects(['recorded_trajectory'])['recorded_trajectory']
                            domain.clear_human_inputs()

                            query_params = {
                                'instruction_text' : 'Press Ok if the recorded trajectory looks good.',
                                'display_type' : 2,
                                'camera_topic' : '/rgb/image_raw',
                                'buttons' : [
                                    {
                                        'name' : 'Ok',
                                        'text' : '',
                                    },
                                    {
                                        'name' : 'Cancel',
                                        'text' : '',
                                    },
                                ],
                                'traj1' : list(recorded_trajectory['skill_state_dict']['q'].flatten())
                            }
                            
                            query_response = domain.run_query_until_done('Teaching 5', query_params)
                            button_inputs = query_response['button_inputs']

                            if button_inputs['Ok'] == 1:
                                domain.clear_human_inputs()
                                recorded_trajectory['duration'] = skill_duration
                                
                                domain.set_memory_objects({skill_name : {'trajectory' : recorded_trajectory}})
                                domain.clear_memory(['recorded_trajectory'])
                                domain.clear_human_inputs()

                            elif button_inputs['Cancel'] == 1:
                                domain.clear_human_inputs()
                        elif button_inputs['Cancel'] == 1:
                            domain.clear_human_inputs()
                    elif button_inputs['Cancel'] == 1:
                        domain.clear_human_inputs()
                elif button_inputs['Cancel'] == 1:
                    domain.clear_human_inputs()
            elif button_inputs['Cancel'] == 1:
                domain.clear_human_inputs()

        elif button_inputs['Replay Trajectory'] == 1:
            domain.clear_human_inputs()

            query_params = {
                'instruction_text' : 'Enter the name of the skill. Then hold onto the robot and press Start.',
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
                'text_inputs' : [
                    {
                        'name' : 'skill_name',
                        'text' : 'Skill Name',
                        'value' : '',
                    },
                ]
            }

            query_response = domain.run_query_until_done('Replay Trajectory 1', query_params)
            button_inputs = query_response['button_inputs']
            text_inputs = query_response['text_inputs']

            if button_inputs['Start'] == 1:
                domain.clear_human_inputs()
                skill_name = text_inputs['skill_name']
                saved_skill_params = domain.get_memory_objects([skill_name])[skill_name]
                if saved_skill_params is not None:

                    skill_params = {
                        'duration' : 10,
                        'dt' : 0.01
                    }
                    skill_id = domain.run_skill('zero_force', json.dumps(skill_params))

                    query_params = {
                        'instruction_text' : 'Move the Robot to the Starting Position and Press Done when Completed.',
                        'buttons' : [
                            {
                                'name' : 'Done',
                                'text' : '',
                            },
                            {
                                'name' : 'Cancel',
                                'text' : '',
                            },
                        ]
                    }
                    query_response = domain.run_query_until_done('Replay Trajectory 2', query_params)
                    button_inputs = query_response['button_inputs']

                    if button_inputs['Done'] == 1:
                        domain.clear_human_inputs()
                        if domain.get_skill_status(skill_id) == 'running':
                            domain.cancel_skill(skill_id)

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
                        query_response = domain.run_query_until_done('Replay Trajectory 3', query_params)
                        button_inputs = query_response['button_inputs']

                        if button_inputs['Start'] == 1:
                            domain.clear_human_inputs()

                            skill_params = {
                                'duration' : 5,
                                'dt' : 0.01,
                                'goal_joints' : list(saved_skill_params['trajectory']['skill_state_dict']['q'][0])
                            }
                            skill_id = domain.run_skill('one_step_joint', json.dumps(skill_params))

                            query_params = {
                                'instruction_text' : 'The robot will first move to the starting point of the saved skill.',
                                'buttons' : [
                                    {
                                        'name' : 'Cancel',
                                        'text' : '',
                                    },
                                ]
                            }
                            query_id = domain.run_query('Replay Trajectory 4', json.dumps(query_params))
                            (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)

                            if skill_done:
                                domain.cancel_query(query_id)
                                domain.clear_human_inputs()
                                time.sleep(1)

                                skill_params = {
                                    'dt' : 0.02,
                                    'traj' : saved_skill_params['trajectory']['skill_state_dict']['q'].tolist()
                                }
                                skill_id = domain.run_skill('stream_joint_traj', json.dumps(skill_params))

                                query_params = {
                                    'instruction_text' : 'The robot will execute the saved skill.',
                                    'buttons' : [
                                        {
                                            'name' : 'Cancel',
                                            'text' : '',
                                        },
                                    ]
                                }
                                query_id = domain.run_query('Replay Trajectory 5', json.dumps(query_params))
                                (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)
                                if skill_done:
                                    domain.cancel_query(query_id)
                                    domain.clear_human_inputs()
                                    time.sleep(1)

                                elif query_done:
                                    button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                    if button_inputs['Cancel'] == 1:
                                        domain.cancel_skill(skill_id)
                                        domain.clear_human_inputs()
                            elif query_done:
                                button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                if button_inputs['Cancel'] == 1:
                                    domain.cancel_skill(skill_id)
                                    domain.clear_human_inputs()

                        elif button_inputs['Cancel'] == 1:
                            domain.clear_human_inputs()
                    elif button_inputs['Cancel'] == 1:
                        domain.clear_human_inputs()        
            elif button_inputs['Cancel'] == 1:
                domain.clear_human_inputs()
        elif button_inputs['Train DMP'] == 1:
            domain.clear_human_inputs()

            query_params = {
                'instruction_text' : 'Enter the name of the skill.',
                'buttons' : [
                    {
                        'name' : 'Submit',
                        'text' : '',
                    },
                    {
                        'name' : 'Cancel',
                        'text' : '',
                    },
                ],
                'text_inputs' : [
                    {
                        'name' : 'skill_name',
                        'text' : 'Skill Name',
                        'value' : '',
                    },
                ]
            }

            query_response = domain.run_query_until_done('Train DMP Skill 1', query_params)
            button_inputs = query_response['button_inputs']
            text_inputs = query_response['text_inputs']

            if button_inputs['Submit'] == 1:
                domain.clear_human_inputs()
                skill_name = text_inputs['skill_name']
                saved_skill_params = domain.get_memory_objects([skill_name])[skill_name]
                if saved_skill_params is not None:

                    recorded_trajectory = saved_skill_params['trajectory']

                    bokeh_traj = {}
                    bokeh_traj['time_since_skill_started'] = list(recorded_trajectory['skill_state_dict']['time_since_skill_started'])
                    bokeh_traj['num_joints'] = 7
                    bokeh_traj['cart_traj'] = list(np.array(recorded_trajectory['skill_state_dict']['O_T_EE']).flatten())
                    bokeh_traj['joint_traj'] = list(recorded_trajectory['skill_state_dict']['q'].flatten())

                    query_params = {
                        'instruction_text' : 'Truncate the trajectory.',
                        'display_type' : 3,
                        'bokeh_display_type' : 0,
                        'bokeh_traj' : bokeh_traj
                    }
                    query_response = domain.run_query_until_done('Train DMP Skill 2', query_params)

                    domain.set_memory_objects({skill_name : {'trajectory' : recorded_trajectory, 
                                                             'dmp_params' : query_response['dmp_params']}})
                    domain.clear_memory(['recorded_trajectory'])
                    domain.clear_human_inputs()
       
            elif button_inputs['Cancel'] == 1:
                domain.clear_human_inputs()
        elif button_inputs['Execute DMP'] == 1:
            domain.clear_human_inputs()

            query_params = {
                'instruction_text' : 'Enter the name of the skill. Then hold onto the robot and press Start.',
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
                'text_inputs' : [
                    {
                        'name' : 'skill_name',
                        'text' : 'Skill Name',
                        'value' : '',
                    },
                ]
            }

            query_response = domain.run_query_until_done('Execute DMP Skill 1', query_params)
            button_inputs = query_response['button_inputs']
            text_inputs = query_response['text_inputs']

            if button_inputs['Start'] == 1:
                domain.clear_human_inputs()
                skill_name = text_inputs['skill_name']
                saved_skill_params = domain.get_memory_objects([skill_name])[skill_name]
                if saved_skill_params is not None:

                    skill_params = {
                        'duration' : 10,
                        'dt' : 0.01
                    }
                    skill_id = domain.run_skill('zero_force', json.dumps(skill_params))

                    query_params = {
                        'instruction_text' : 'Move the Robot to the Starting Position and Press Done when Completed.',
                        'buttons' : [
                            {
                                'name' : 'Done',
                                'text' : '',
                            },
                            {
                                'name' : 'Cancel',
                                'text' : '',
                            },
                        ]
                    }
                    query_response = domain.run_query_until_done('Execute DMP Skill 2', query_params)
                    button_inputs = query_response['button_inputs']

                    if button_inputs['Done'] == 1:
                        domain.clear_human_inputs()
                        if domain.get_skill_status(skill_id) == 'running':
                            domain.cancel_skill(skill_id)

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
                        query_response = domain.run_query_until_done('Execute DMP Skill 3', query_params)
                        button_inputs = query_response['button_inputs']

                        if button_inputs['Start'] == 1:
                            domain.clear_human_inputs()

                            if saved_skill_params['dmp_params']['dmp_type'] == 0:
                                skill_params = {
                                    'duration' : 5,
                                    'dt' : 0.01,
                                    'position_dmp_params' : saved_skill_params['dmp_params'],
                                    'quat_dmp_params' : saved_skill_params['dmp_params']['quat_dmp_params']
                                }
                                skill_id = domain.run_skill('one_step_quat_pose_dmp', json.dumps(skill_params))
                            else:
                                skill_params = {
                                    'duration' : 5,
                                    'dt' : 0.01,
                                    'dmp_params' : saved_skill_params['dmp_params']
                                }
                                skill_id = domain.run_skill('one_step_joint_dmp', json.dumps(skill_params))

                            query_params = {
                                'instruction_text' : 'The robot will execute the saved skill.',
                                'buttons' : [
                                    {
                                        'name' : 'Cancel',
                                        'text' : '',
                                    },
                                ]
                            }
                            query_id = domain.run_query('Execute DMP Skill 4', json.dumps(query_params))
                            (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)
                            if skill_done:
                                domain.cancel_query(query_id)
                                domain.clear_human_inputs()
                                time.sleep(1)

                            elif query_done:
                                button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                if button_inputs['Cancel'] == 1:
                                    domain.cancel_skill(skill_id)
                                    domain.clear_human_inputs()

                        elif button_inputs['Cancel'] == 1:
                            domain.clear_human_inputs()
                    elif button_inputs['Cancel'] == 1:
                        domain.clear_human_inputs()        
            elif button_inputs['Cancel'] == 1:
                domain.clear_human_inputs()
        elif button_inputs['Record Point Trajectory'] == 1:
            domain.clear_human_inputs()

            query_params = {
                'instruction_text' : 'Enter the name of the point trajectory. Then hold onto the robot and press Start to move the robot to the initial point.',
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
                'text_inputs' : [
                    {
                        'name' : 'point_trajectory_name',
                        'text' : 'Point Trajectory Name',
                        'value' : '',
                    },
                ]
            }

            query_response = domain.run_query_until_done('Record Point Trajectory 1', query_params)
            button_inputs = query_response['button_inputs']
            text_inputs = query_response['text_inputs']

            if button_inputs['Start'] == 1:
                domain.clear_human_inputs()
                point_trajectory_name = text_inputs['point_trajectory_name']

                skill_params = {
                    'duration' : 10,
                    'dt' : 0.01
                }
                skill_id = domain.run_skill('zero_force', json.dumps(skill_params))

                query_params = {
                    'instruction_text' : 'Move the Robot to the Starting Position and Press Done when Completed.',
                    'buttons' : [
                        {
                            'name' : 'Done',
                            'text' : '',
                        },
                        {
                            'name' : 'Cancel',
                            'text' : '',
                        },
                    ]
                }
                query_response = domain.run_query_until_done('Record Point Trajectory 2', query_params)
                button_inputs = query_response['button_inputs']

                if button_inputs['Done'] == 1:
                    domain.clear_human_inputs()
                    if domain.get_skill_status(skill_id) == 'running':
                        domain.cancel_skill(skill_id)
                        time.sleep(1)

                    ee_positions = []
                    ee_quaternions = []
                    joint_positions = []

                    current_state = domain.state
                    ee_positions.append(current_state['frame:franka:ee:pose/position'])
                    ee_quaternions.append(current_state['frame:franka:ee:pose/quaternion'])
                    joint_positions.append(current_state['frame:franka:joints/position'])

                    skill_params = {
                        'duration' : 10,
                        'dt' : 0.01
                    }
                    skill_id = domain.run_skill('zero_force', json.dumps(skill_params))

                    query_params = {
                        'instruction_text' : 'Press Next when you have finished moving the robot to the next point. Otherwise Press Done when the trajectory is finished.',
                        'buttons' : [
                            {
                                'name' : 'Next',
                                'text' : '',
                            },
                            {
                                'name' : 'Done',
                                'text' : '',
                            },
                            {
                                'name' : 'Cancel',
                                'text' : '',
                            },
                        ],
                    }

                    point_num = 1

                    query_response = domain.run_query_until_done('Record Points '+str(point_num), query_params)
                    button_inputs = query_response['button_inputs']

                    while button_inputs['Next'] == 1:
                        domain.clear_human_inputs()
                        if domain.get_skill_status(skill_id) == 'running':
                            domain.cancel_skill(skill_id)
                            time.sleep(1)

                        current_state = domain.state
                        ee_positions.append(current_state['frame:franka:ee:pose/position'])
                        ee_quaternions.append(current_state['frame:franka:ee:pose/quaternion'])
                        joint_positions.append(current_state['frame:franka:joints/position'])

                        point_num += 1

                        skill_id = domain.run_skill('zero_force', json.dumps(skill_params))

                        query_response = domain.run_query_until_done('Record Points '+str(point_num), query_params)
                        button_inputs = query_response['button_inputs']

                    if button_inputs['Done'] == 1:
                        domain.clear_human_inputs()
                        if domain.get_skill_status(skill_id) == 'running':
                            domain.cancel_skill(skill_id)
                            time.sleep(1)

                        current_state = domain.state
                        ee_positions.append(current_state['frame:franka:ee:pose/position'])
                        ee_quaternions.append(current_state['frame:franka:ee:pose/quaternion'])
                        joint_positions.append(current_state['frame:franka:joints/position'])

                        domain.set_memory_objects({point_trajectory_name : {'ee_positions' : ee_positions, 
                                                                            'ee_quaternions' : ee_quaternions,
                                                                            'joint_positions' : joint_positions}})
                        
                    elif button_inputs['Cancel'] == 1:
                        domain.clear_human_inputs()
                elif button_inputs['Cancel'] == 1:
                    domain.clear_human_inputs()             
            elif button_inputs['Cancel'] == 1:
                domain.clear_human_inputs()

        elif button_inputs['Replay Point Trajectory'] == 1:
            domain.clear_human_inputs()

            query_params = {
                'instruction_text' : 'Enter the name of the point trajectory. Then hold onto the robot and press Start to move the robot to the initial point.',
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
                'text_inputs' : [
                    {
                        'name' : 'point_trajectory_name',
                        'text' : 'Point Trajectory Name',
                        'value' : '',
                    },
                ]
            }

            query_response = domain.run_query_until_done('Replay Point Trajectory 1', query_params)
            button_inputs = query_response['button_inputs']
            text_inputs = query_response['text_inputs']

            if button_inputs['Start'] == 1:
                domain.clear_human_inputs()
                point_trajectory_name = text_inputs['point_trajectory_name']
                saved_point_trajectory_points = domain.get_memory_objects([point_trajectory_name])[point_trajectory_name]
                if saved_point_trajectory_points is not None:

                    skill_params = {
                        'duration' : 10,
                        'dt' : 0.01
                    }
                    skill_id = domain.run_skill('zero_force', json.dumps(skill_params))

                    query_params = {
                        'instruction_text' : 'Move the Robot to the Starting Position and Press Done when Completed.',
                        'buttons' : [
                            {
                                'name' : 'Done',
                                'text' : '',
                            },
                            {
                                'name' : 'Cancel',
                                'text' : '',
                            },
                        ]
                    }
                    query_response = domain.run_query_until_done('Replay Point Trajectory 2', query_params)
                    button_inputs = query_response['button_inputs']

                    if button_inputs['Done'] == 1:
                        domain.clear_human_inputs()
                        if domain.get_skill_status(skill_id) == 'running':
                            domain.cancel_skill(skill_id)

                        query_params = {
                            'instruction_text' : 'Press Joint or End Effector to start the trajectory replay when you are safely away from the robot.',
                            'buttons' : [
                                {
                                    'name' : 'Joint',
                                    'text' : '',
                                },
                                {
                                    'name' : 'End Effector',
                                    'text' : '',
                                },
                                {
                                    'name' : 'Cancel',
                                    'text' : '',
                                },
                            ],
                        }
                        query_response = domain.run_query_until_done('Replay Point Trajectory 3', query_params)
                        button_inputs = query_response['button_inputs']

                        if button_inputs['Joint'] == 1:
                            domain.clear_human_inputs()

                            skill_params = {
                                'duration' : 5,
                                'dt' : 0.01,
                                'goal_joints' : list(saved_point_trajectory_points['joint_positions'][0])
                            }
                            skill_id = domain.run_skill('one_step_joint', json.dumps(skill_params))

                            query_params = {
                                'instruction_text' : 'The robot will first move to the starting point of the saved skill.',
                                'buttons' : [
                                    {
                                        'name' : 'Cancel',
                                        'text' : '',
                                    },
                                ]
                            }
                            query_id = domain.run_query('Replay Point Trajectory 4', json.dumps(query_params))
                            (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)

                            if skill_done:
                                domain.cancel_query(query_id)
                                domain.clear_human_inputs()
                                time.sleep(1)

                                query_params = {
                                    'instruction_text' : 'The robot will iterate through all of the saved trajectory points.',
                                    'buttons' : [
                                        {
                                            'name' : 'Cancel',
                                            'text' : '',
                                        },
                                    ]
                                }
                                query_id = domain.run_query('Replay Point Trajectory 5', json.dumps(query_params))

                                for point_num in range(1,len(saved_point_trajectory_points['joint_positions'])):
                                    skill_params = {
                                        'duration' : 5,
                                        'dt' : 0.01,
                                        'goal_joints' : list(saved_point_trajectory_points['joint_positions'][point_num])
                                    }
                                    skill_id = domain.run_skill('one_step_joint', json.dumps(skill_params))

                                
                                    (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)
                                    if skill_done:
                                        continue

                                    elif query_done:
                                        button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                        if button_inputs['Cancel'] == 1:
                                            if domain.get_skill_status(skill_id) == 'running':
                                                domain.cancel_skill(skill_id)
                                            domain.clear_human_inputs()
                                            break
                                domain.cancel_query(query_id)
                                domain.clear_human_inputs()
                                time.sleep(1)
                            elif query_done:
                                button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                if button_inputs['Cancel'] == 1:
                                    if domain.get_skill_status(skill_id) == 'running':
                                        domain.cancel_skill(skill_id)
                                    domain.clear_human_inputs()
                        elif button_inputs['End Effector'] == 1:
                            domain.clear_human_inputs()

                            ee_pose = RigidTransform(rotation=qt.as_rotation_matrix(qt.from_float_array(np.array(saved_point_trajectory_points['ee_quaternions'][0]))), 
                                                     translation=np.array(saved_point_trajectory_points['ee_positions'][0]))

                            skill_params = {
                                'duration' : 5,
                                'dt' : 0.01,
                                'goal_pose' : list(convert_rigid_transform_to_array(ee_pose))
                            }
                            skill_id = domain.run_skill('one_step_pose', json.dumps(skill_params))

                            query_params = {
                                'instruction_text' : 'The robot will first move to the starting point of the saved skill.',
                                'buttons' : [
                                    {
                                        'name' : 'Cancel',
                                        'text' : '',
                                    },
                                ]
                            }
                            query_id = domain.run_query('Replay Point Trajectory 4', json.dumps(query_params))
                            (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)

                            if skill_done:
                                domain.cancel_query(query_id)
                                domain.clear_human_inputs()
                                time.sleep(1)

                                query_params = {
                                    'instruction_text' : 'The robot will iterate through all of the saved trajectory points.',
                                    'buttons' : [
                                        {
                                            'name' : 'Cancel',
                                            'text' : '',
                                        },
                                    ]
                                }
                                query_id = domain.run_query('Replay Point Trajectory 5', json.dumps(query_params))

                                for point_num in range(1,len(saved_point_trajectory_points['ee_positions'])):
                                    ee_pose = RigidTransform(rotation=qt.as_rotation_matrix(qt.from_float_array(np.array(saved_point_trajectory_points['ee_quaternions'][point_num]))), 
                                                             translation=np.array(saved_point_trajectory_points['ee_positions'][point_num]))

                                    skill_params = {
                                        'duration' : 5,
                                        'dt' : 0.01,
                                        'goal_pose' : list(convert_rigid_transform_to_array(ee_pose))
                                    }
                                    skill_id = domain.run_skill('one_step_pose', json.dumps(skill_params))

                                
                                    (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)
                                    if skill_done:
                                        continue

                                    elif query_done:
                                        button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                        if button_inputs['Cancel'] == 1:
                                            if domain.get_skill_status(skill_id) == 'running':
                                                domain.cancel_skill(skill_id)
                                            domain.clear_human_inputs()
                                            break
                                domain.cancel_query(query_id)
                                domain.clear_human_inputs()
                                time.sleep(1)
                            elif query_done:
                                button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                if button_inputs['Cancel'] == 1:
                                    if domain.get_skill_status(skill_id) == 'running':
                                        domain.cancel_skill(skill_id)
                                    domain.clear_human_inputs()


                        elif button_inputs['Cancel'] == 1:
                            domain.clear_human_inputs()
                    elif button_inputs['Cancel'] == 1:
                        domain.clear_human_inputs()        
            elif button_inputs['Cancel'] == 1:
                domain.clear_human_inputs()
                
        elif button_inputs['Pick Up Object'] == 1:
            domain.clear_human_inputs()

            (rgb_image_success, rgb_image_path) = domain.save_rgb_camera_image('/rgb/image_raw')
            depth_image_path = rgb_image_path[:rgb_image_path.rfind('/')+1] + 'depth_' + rgb_image_path[rgb_image_path.rfind('/')+1:]
            (depth_image_success, depth_image_path) = domain.save_depth_camera_image('/depth_to_rgb/image_raw', depth_image_path)

            (success, image_path, image) = domain.get_rgb_image(rgb_image_path)

            if success:

                query_params = {
                    'instruction_text' : 'Click points on the image corresponding to goal locations for an object. Press submit when done.',
                    'display_type' : 3,
                    'bokeh_display_type' : 2,
                    'bokeh_image' : image.tolist(),
                }

                query_response = domain.run_query_until_done('Pick Up Object 1', query_params)

                (goal_points_success, goal_points) = domain.get_goal_points(depth_image_path, query_response['desired_positions'])

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

                    query_params = {
                        'instruction_text' : 'Press Cancel if you would like to stop the robot.',
                        'buttons' : [
                            {
                                'name' : 'Cancel',
                                'text' : '',
                            },
                        ]
                    }
                    query_id = domain.run_query('Pick and Place 2', json.dumps(query_params))
                    (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)

                    if skill_done:
                        goal_pose_1 = goal_points[0]
                        intermediate_height_offset = 0.11
                        intermediate_pose = RigidTransform(rotation=np.array([
                            [1, 0, 0],
                            [0, -1, 0],
                            [0, 0, -1],
                        ]), translation=np.array([goal_pose_1.x, goal_pose_1.y, goal_pose_1.z + intermediate_height_offset]))

                        grasp_pose = RigidTransform(rotation=np.array([
                            [1, 0, 0],
                            [0, -1, 0],
                            [0, 0, -1],
                        ]), translation=np.array([goal_pose_1.x, goal_pose_1.y, goal_pose_1.z]))

                        skill_params = {
                            'duration' : 5,
                            'dt' : 0.01,
                            'goal_pose' : list(convert_rigid_transform_to_array(intermediate_pose))
                        }
                        skill_id = domain.run_skill('one_step_pose', json.dumps(skill_params))

                        (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)
                        if skill_done:
                            skill_params = {
                                'duration' : 5,
                                'dt' : 0.01,
                                'goal_pose' : list(convert_rigid_transform_to_array(grasp_pose))
                            }
                            skill_id = domain.run_skill('one_step_pose', json.dumps(skill_params))
                            (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)
                            if skill_done:
                                skill_id = domain.run_skill('close_gripper', '')
                                (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)
                                if skill_done:
                                    skill_params = {
                                        'duration' : 5,
                                        'dt' : 0.01,
                                        'goal_pose' : list(convert_rigid_transform_to_array(intermediate_pose))
                                    }
                                    skill_id = domain.run_skill('one_step_pose', json.dumps(skill_params))
                                    (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)
                                    if skill_done:
                                        skill_params = {
                                            'duration' : 5,
                                            'dt' : 0.01,
                                            'goal_pose' : list(convert_rigid_transform_to_array(grasp_pose))
                                        }
                                        skill_id = domain.run_skill('one_step_pose', json.dumps(skill_params))
                                        (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)
                                        if skill_done:
                                            skill_id = domain.run_skill('open_gripper', '')
                                            (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)
                                            if skill_done:
                                                skill_params = {
                                                    'duration' : 5,
                                                    'dt' : 0.01,
                                                    'goal_pose' : list(convert_rigid_transform_to_array(intermediate_pose))
                                                }
                                                skill_id = domain.run_skill('one_step_pose', json.dumps(skill_params))
                                                (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)
                                                if skill_done:
                                                    skill_params = {
                                                        'duration' : 5,
                                                        'dt' : 0.01,
                                                        'goal_joints' : [0, -math.pi / 4, 0, -3 * math.pi / 4, 0, math.pi / 2, math.pi / 4]
                                                    }
                                                    skill_id = domain.run_skill('one_step_joint', json.dumps(skill_params))
                                                    (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)

                                                    if skill_done:
                                                        domain.cancel_query(query_id)
                                                        domain.clear_human_inputs()

                                                    elif query_done:
                                                        button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                                        if button_inputs['Cancel'] == 1:
                                                            domain.cancel_skill(skill_id)
                                                            domain.clear_human_inputs()

                                                elif query_done:
                                                    button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                                    if button_inputs['Cancel'] == 1:
                                                        domain.cancel_skill(skill_id)
                                                        domain.clear_human_inputs()
                                            elif query_done:
                                                button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                                if button_inputs['Cancel'] == 1:
                                                    domain.cancel_skill(skill_id)
                                                    domain.clear_human_inputs()
                                        elif query_done:
                                            button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                            if button_inputs['Cancel'] == 1:
                                                domain.cancel_skill(skill_id)
                                                domain.clear_human_inputs()
                                    elif query_done:
                                        button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                        if button_inputs['Cancel'] == 1:
                                            domain.cancel_skill(skill_id)
                                            domain.clear_human_inputs()
                                elif query_done:
                                    button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                    if button_inputs['Cancel'] == 1:
                                        domain.cancel_skill(skill_id)
                                        domain.clear_human_inputs()
                            elif query_done:
                                button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                                if button_inputs['Cancel'] == 1:
                                    domain.cancel_skill(skill_id)
                                    domain.clear_human_inputs()
                        elif query_done:
                            button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                            if button_inputs['Cancel'] == 1:
                                domain.cancel_skill(skill_id)
                                domain.clear_human_inputs()
                    elif query_done:
                        button_inputs = domain.get_memory_objects(['buttons'])['buttons']

                        if button_inputs['Cancel'] == 1:
                            domain.cancel_skill(skill_id)
                            domain.clear_human_inputs()

                elif button_inputs['Cancel'] == 1:
                    domain.clear_human_inputs()