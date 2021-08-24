import rospy
import json
import time

from iam_domain_handler.domain_client import DomainClient

if __name__ == '__main__':
    #rospy.init_node('run_domain_client')
    domain = DomainClient()

    while not rospy.is_shutdown():
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
            ]
        }
        query_id = domain.run_query('Initial Options', json.dumps(query_params))
        domain.wait_until_query_done(query_id)

        button_inputs = domain.get_memory_objects(['buttons'])['buttons']
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
            query_id = domain.run_query('Teaching 1', json.dumps(query_params))
            domain.wait_until_query_done(query_id)

            button_inputs = domain.get_memory_objects(['buttons'])['buttons']
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
                query_id = domain.run_query('Teaching 2', json.dumps(query_params))
                domain.wait_until_query_done(query_id)

                button_inputs = domain.get_memory_objects(['buttons'])['buttons']
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
                    query_id = domain.run_query('Teaching 3', json.dumps(query_params))
                    domain.wait_until_query_done(query_id)

                    button_inputs = domain.get_memory_objects(['buttons'])['buttons']
                    text_inputs = domain.get_memory_objects(['text_inputs'])['text_inputs']
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

                        query_id = domain.run_query('Teaching 4', json.dumps(query_params))
                        domain.wait_until_query_done(query_id)

                        button_inputs = domain.get_memory_objects(['buttons'])['buttons']
                        if button_inputs['Done'] == 1:
                            recorded_trajectory = domain.get_memory_objects(['recorded_trajectory'])['recorded_trajectory']
                            domain.clear_human_inputs()

                            query_params = {
                                'instruction_text' : 'Press Ok if the recorded trajectory looks good.',
                                'display_type' : 2,
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

                            query_id = domain.run_query('Teaching 5', json.dumps(query_params))
                            domain.wait_until_query_done(query_id)
                            button_inputs = domain.get_memory_objects(['buttons'])['buttons']
                            if button_inputs['Ok'] == 1:
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

        elif button_inputs['Execute Skill'] == 1:
            domain.clear_human_inputs()