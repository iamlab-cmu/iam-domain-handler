import rospy
import json
import time

from iam_domain_handler.domain_client import DomainClient

if __name__ == '__main__':
    rospy.init_node('run_domain_client')
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
                domain.run_skill()

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
                elif button_inputs['Cancel'] == 1:
                    domain.clear_human_inputs()
            elif button_inputs['Cancel'] == 1:
                domain.clear_human_inputs()

        elif button_inputs['Execute Skill'] == 1:
            domain.clear_human_inputs()