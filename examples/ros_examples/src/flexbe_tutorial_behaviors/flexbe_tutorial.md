# flexbe_tutorial_behaviors - So far one of the best documented ros packages I've ever seen
0. Basic Functions
    - How does it know which flexbe repos are there?
    - There's a state file, defining a type of state; also a statemachine file, that **put the states together, and create inputs for them**
    - Define states, behaviors, then run them: ```roslaunch flexbe_app flexbe_full.launc```
        - just the user interface only `roslaunch flexbe_app flexbe_ocs.launch`
1. Flexbe key functions
    - change autonomy level. Autonomy level is no, low, high, full? 
    - Create private variables
    - **code generation as we add states**
2. Cool add-ons 
    - Auto complete and search 
    - How did it know which states are available?
3. states
    - we're overriding EventState class, one state per file.
    - execute is executed 10hz by default
        - **Should be non-blocking so a remote operator can pause any time**
    - userdata: input, and output [example](https://github.com/RicoJia/notes/blob/master/examples/ros_examples/src/flexbe_tutorial_behaviors/flexbe_tutorial_flexbe_states/src/flexbe_tutorial_flexbe_states/example_state.py)
        - Input of a state can come from either the state machine, when we pass information in, or from the previous state (```userdata```) 
        - One caveat is that you cannot change the inputs of the system, this might be an issue for reusing the state class.
    - a state provides an input, and output. For navstack, you might want to have a giant state to handle all the internal states. For moveit, where planning and execution are different states, you can have different states
        
    - **state transition:** [link](http://wiki.ros.org/flexbe/Tutorials/The%20State%20Lifecycle)
4. Code Structure
    ```
        ├── flexbe_tutorial_flexbe_behaviors
    │   ├── CMakeLists.txt
    │   ├── config
    │   │   └── example.yaml (What for?)
    │   ├── manifest
    │   │   ├── example_behavior.xml
    │   │   └── hello_overview.xml  #"exports a module in the package"
    │   ├── package.xml
    │   ├── setup.py        #(for setting up the package?)
    │   └── src     #(this is actually a package already)
    │       └── flexbe_tutorial_flexbe_behaviors
    │           ├── example_behavior_sm.py
    │           ├── hello_overview_sm.py
    │           └── __init__.py
    ├── flexbe_tutorial_flexbe_states
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── setup.py
    │   └── src
    │       └── flexbe_tutorial_flexbe_states
    │           ├── example_action_state.py
    │           ├── example_state2.py
    │           ├── example_state.py
    │           ├── __init__.py
    │           └── __pycache__
    │               ├── example_state2.cpython-38.pyc
    │               └── example_state.cpython-38.pyc
    ├── flexbe_tutorial.md
    └── LICENSE
    ```
    

## Intermediate Uses
1. Proxy, [link](http://wiki.ros.org/flexbe/Tutorials/Developing%20Basic%20States)
    - a proxy will handle a bunch of actions, topics, so you pass in a dict. Also when you use it, you should do 
    specify which action server you're using
    - But I think you can just instantiate an action server / publisher
2. Concurrency containers for parallel execution 
3. You can add an action server to the flexbe engine: [link](    http://wiki.ros.org/flexbe/Tutorials/Running%20Behaviors%20Without%20Operator)
4. Make an actionserver that synthesizes states. Flexbe can have a gui that passes in a message. [link](http://wiki.ros.org/flexbe/Tutorials/Behavior%20Synthesis%20Interface)
    - Do this when you create the state machine
    - for action result, send `StateInstantiation` back

## Quirks:
1. configure your editor to use tabs only, Python will get angry at spaces 
2. When you load behavior, changes in behavior will update, but states are not

========================================================================
## How to launch 
========================================================================
1. Link: http://wiki.ros.org/flexbe/Tutorials/Running%20Behaviors%20Without%20Operator
    - Launch behavior_onboard: 
        1. 1st way ```roslaunch flexbe_onboard behavior_onboard.launch```
        2. 2nd way: 
            ```python
            from flexbe_core.proxy import ProxySubscriberCached
            from flexbe_onboard.flexbe_onboard import FlexbeOnboard 
            if __name__ == '__main__':
                rospy.init_node('flexbe_onboard')
                FlexbeOnboard()
                # Wait for ctrl-c to stop the application
                rospy.spin()
                ProxySubscriberCached().shutdown()
            ```
    - Launch behavior
        1. 1st way ```rosrun flexbe_widget be_launcher -b 'Example Behavior'``` on one panel
        2. 2nd way: ```rosrun flexbe_widget be_action_server```, then publish behavior name onto ```rostopic pub /flexbe/execute_behavior/goal flexbe_msgs/BehaviorExecutionActionGoal '{goal: {behavior_name: "Example Behavior"}}'```
