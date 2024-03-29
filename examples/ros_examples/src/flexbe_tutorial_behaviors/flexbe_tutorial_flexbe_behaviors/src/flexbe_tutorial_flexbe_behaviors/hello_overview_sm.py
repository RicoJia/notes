#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.log_state import LogState as flexbe_states__LogState
from flexbe_tutorial_flexbe_states.example_state import ExampleState as flexbe_tutorial_flexbe_states__ExampleState
from flexbe_tutorial_flexbe_states.example_state2 import ExampleState2 as flexbe_tutorial_flexbe_states__ExampleState2
from flexbe_tutorial_flexbe_states.example_state3 import ExampleState3 as flexbe_tutorial_flexbe_states__ExampleState3
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Aug 14 2022
@author: RJ
'''
class hello_overviewSM(Behavior):
    '''
    Test disc
    '''


    def __init__(self):
        super(hello_overviewSM, self).__init__()
        self.name = 'hello_overview'

        # parameters of this behavior
        self.add_parameter('wait_time', 1)
        self.add_parameter('print_persistent_member', 0)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        hello = "Hello World"
        # x:30 y:365, x:130 y:365
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.hello_world_msg = "hola como estas"

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:107 y:26
            OperatableStateMachine.add('Wait',
                                        flexbe_tutorial_flexbe_states__ExampleState(target_time=self.wait_time),
                                        transitions={'continue': 'wait2', 'failed': 'wait2'},
                                        autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'hello_world_msg': 'hello_world_msg', 'test_output': 'test_output', 'hello_world_msg2': 'hello_world_msg2'})

            # x:30 y:88
            OperatableStateMachine.add('example3',
                                        flexbe_tutorial_flexbe_states__ExampleState3(target_time=0.4, persistent_member=self.print_persistent_member),
                                        transitions={'continue': 'print_greeting', 'failed': 'print_greeting'},
                                        autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'hello_world_msg3': 'hello_world_msg', 'dummy_input': 'hello_world_msg2', 'test_output': 'test_output'})

            # x:107 y:176
            OperatableStateMachine.add('print_greeting',
                                        flexbe_states__LogState(text=hello, severity=Logger.REPORT_HINT),
                                        transitions={'done': 'finished'},
                                        autonomy={'done': Autonomy.High})

            # x:375 y:31
            OperatableStateMachine.add('wait2',
                                        flexbe_tutorial_flexbe_states__ExampleState2(target_time=self.wait_time),
                                        transitions={'continue': 'example3', 'failed': 'print_greeting'},
                                        autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
                                        remapping={'hello_world_msg2': 'hello_world_msg2', 'test_output': 'test_output'})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
