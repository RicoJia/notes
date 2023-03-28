#!/usr/bin/env python
import rospy
import numpy as np
import json 
from flexbe_core import EventState, Logger


class ExampleState3(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self, target_time, persistent_member):

		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(ExampleState3, self).__init__(outcomes = ['continue', 'failed'], input_keys=["hello_world_msg3", "dummy_input"], output_keys=["test_output"])

		# Store state parameter for later use.
		self._target_time = rospy.Duration(target_time) 
		self.persistent_member = persistent_member 
		# The constructor is called when building the state machine, not when actually starting the behavior.
		# Thus, we cannot save the starting time now and will do so later.
		self._start_time = None


	def execute(self, userdata):
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		# Otherwise this method is called periodically (10hz) while the state is active.  
		# print("haha", userdata["hello_world_msg3"])
		userdata["test_output"] = 12345
		self.persistent_member
		json.dumps({1: np.ndarray((2,), buffer=np.array([1,2,3]),dtype=int)})
		print("HAHAHAH")
		if rospy.Time.now() - self._start_time > self._target_time:
			return 'continue' # One of the outcomes declared above.
		

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		# The following code is just for illustrating how the behavior logger works.
		# Text logged by the behavior logger is sent to the operator and displayed in the GUI.

		self._start_time = rospy.Time.now()
		time_to_wait = (self._target_time - (rospy.Time.now() - self._start_time)).to_sec()

		if time_to_wait > 0:
			Logger.loginfo('Need to wait for %.1f seconds.' % time_to_wait)


	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.
		print("On exit: %s" % userdata["test_output"])
		pass # Nothing to do in this example.


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		# In this example, we use this event to set the correct start time.
		pass

	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.
		
