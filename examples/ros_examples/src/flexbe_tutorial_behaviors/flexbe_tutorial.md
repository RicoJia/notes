# flexbe_tutorial_behaviors
0. Basic Functions
    - How does it know which flexbe repos are there?
    - There's a state file, defining a type of state; also a statemachine file, that **put the states together, and create inputs for them**
1. Flexbe key functions
    - change autonomy level. Autonomy level is no, low, high, full? 
    - Create private variables
2. Cool add-ons 
    - Auto complete and search 
    - How did it know which states are available?
3. states
    - we're overriding EventState class, one class per file.
    - execute is executed 10hz by defaul
    - userdata: input, and output [example](https://github.com/RicoJia/notes/blob/master/examples/ros_examples/src/flexbe_tutorial_behaviors/flexbe_tutorial_flexbe_states/src/flexbe_tutorial_flexbe_states/example_state.py)
        - Input of a state can come from either the state machine, when we pass information in, or from the previous state  
        - One caveat is that you cannot change the inputs of the system, only output. 

        
    -**state transition: ** [link](http://wiki.ros.org/flexbe/Tutorials/The%20State%20Lifecycle)

    
    