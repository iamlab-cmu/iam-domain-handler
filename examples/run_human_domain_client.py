import rospy
import json
import time

from iam_domain_handler.domain_client import DomainClient

if __name__ == '__main__':
    rospy.init_node('run_domain_client')
    domain = DomainClient()

    while not rospy.is_shutdown():

        button_inputs = []
        text_inputs = []

        query_params = {
            'buttons' : [
                {
                    'name' : 'Teach Skill',
                    'text' : '',
                },
                {
                    'name' : 'Execute Skill',
                    'text' : '',
                },
                {
                    'name' : 'Save Images',
                    'text' : '',
                },
                {
                    'name' : 'Label Images',
                    'text' : '',
                },
                {
                    'name' : 'Select Point Goals',
                    'text' : '',
                },
            ]
        }

        query_response = domain.run_query_until_done('Starting Menu', query_params)
        button_inputs = query_response['button_inputs']

        if button_inputs['Teach Skill'] == 1:
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
                                domain.set_memory_objects({skill_name : recorded_trajectory})
                                domain.clear_memory(['recorded_trajectory'])

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

        elif button_inputs['Execute Skill'] == 1:
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

            query_response = domain.run_query_until_done('Execute Skill 1', query_params)
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
                    query_response = domain.run_query_until_done('Execute Skill 2', query_params)
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
                        query_response = domain.run_query_until_done('Execute Skill 3', query_params)
                        button_inputs = query_response['button_inputs']

                        if button_inputs['Start'] == 1:
                            domain.clear_human_inputs()

                            skill_params = {
                                'duration' : 5,
                                'dt' : 0.01,
                                'goal_joints' : list(saved_skill_params['skill_state_dict']['q'][0])
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
                            query_id = domain.run_query('Execute Skill 4', json.dumps(query_params))
                            (skill_done, query_done) = domain.wait_until_skill_or_query_done(skill_id, query_id)

                            if skill_done:
                                domain.cancel_query(query_id)
                                domain.clear_human_inputs()

                                skill_params = {
                                    'dt' : 0.02,
                                    'traj' : saved_skill_params['skill_state_dict']['q'].tolist()
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
                                query_id = domain.run_query('Execute Skill 5', json.dumps(query_params))
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

                        elif button_inputs['Cancel'] == 1:
                            domain.clear_human_inputs()
                    elif button_inputs['Cancel'] == 1:
                        domain.clear_human_inputs()        
            elif button_inputs['Cancel'] == 1:
                domain.clear_human_inputs()
        elif button_inputs['Save Images'] == 1:
            domain.clear_human_inputs()

            query_params = {
                'instruction_text' : 'Press Save when you want to save images. Else press Done when you have finished.',
                'buttons' : [
                    {
                        'name' : 'Save',
                        'text' : '',
                    },
                    {
                        'name' : 'Done',
                        'text' : '',
                    },
                ]
            }

            image_num = 0

            query_response = domain.run_query_until_done('Save Images '+str(image_num), query_params)
            button_inputs = query_response['button_inputs']

            while button_inputs['Save'] == 1:
                domain.clear_human_inputs()

                domain.save_image('/rgb/image_raw')

                image_num += 1
                query_response = domain.run_query_until_done('Save Images '+str(image_num), query_params)
                button_inputs = query_response['button_inputs']

            if button_inputs['Done'] == 1:
                domain.clear_human_inputs()

        elif button_inputs['Label Images'] == 1:
            domain.clear_human_inputs()

            (success, image_path, image) = domain.get_image()

            if success:
            
                query_params = {
                    'instruction_text' : 'Label the image. Press submit when you are done labeling an image.',
                    'display_type' : 3,
                }
                query_id = domain.run_query('Label Image', json.dumps(query_params))

                time.sleep(1)

                domain.label_image(image)

                query_response = domain.wait_until_query_done(query_id)

        elif button_inputs['Select Point Goals'] == 1:
            domain.clear_human_inputs()

            (success, image_path, image) = domain.get_image()

            if success:
            
                query_params = {
                    'instruction_text' : 'Click points on the image corresponding to goal locations for an object. Press submit when done.',
                    'display_type' : 3,
                }
                query_id = domain.run_query('Get Point Goals', json.dumps(query_params))

                time.sleep(1)

                domain.get_point_goals(image)

                query_response = domain.wait_until_query_done(query_id)


