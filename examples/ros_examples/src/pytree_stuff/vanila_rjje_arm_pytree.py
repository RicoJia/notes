#!/usr/bin/env python3
import py_trees

PICKUP_LOCATION = "pickup_location"
class MoveToStandoffPos(py_trees.behaviour.Behaviour):
    """
    1. __init__ only contains stuff for rendering to offline dot graph
    2. setup() is used for onetime initialization, like hardware. Run only once
    3. initialise(self): everytime we get into this behavior
    4. Update: called everytime ticks
    5. Terminate: when behavior is not running anymore. 
        - you get the new status, after terminate, new status will overwrite the status
        - When behavior fails, terminate is called again to set status to INVALID
        - If succeeds, terminate is set to success, and new behavior starts, executes for 1 tick
    """
    def __init__(self, name, test_blackboard=False):
        super().__init__(name=name)
        self.test_blackboard = test_blackboard
        
    def setup(self):
        self.logger.warning(f"self.__class__.__name__ is set up")
        self.count = 0
        if self.test_blackboard:
            self.blackboard = self.attach_blackboard_client()
            self.blackboard.register_key(key = PICKUP_LOCATION, access=py_trees.common.Access.READ) 

    def initialise(self):
        if self.test_blackboard:
            self.destination = self.blackboard.get(PICKUP_LOCATION) 
            self.logger.info(f"Beginning a new behavior execution: {self.__class__.__name__}"
                            f"Destination: {self.destination}")

    def update(self):
        self.count += 1
        if self.count % 3 == 0: 
            self.logger.info("count is a multiple of 3, failed")
            return py_trees.common.Status.FAILURE
        elif self.count % 5 == 0:
            self.logger.info("Multiple of 5")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.info("Running")
            return py_trees.common.Status.RUNNING
    def terminate(self, new_status):
        self.logger.info(f"terminate: current status: {self.status}, new_status: {new_status}")

class Grasp(py_trees.behaviour.Behaviour):
    def setup(self):
        self.logger.warning(f"self.__class__.__name__ is set up")
        self.count = 0

    def initialise(self):
        self.logger.info("Initialized" + self.__class__.__name__)
    def update(self):
        self.count += 1
        if self.count % 3 != 0: 
            self.logger.info(self.__class__.__name__ + " running")
            return py_trees.common.Status.RUNNING
        else:
            self.logger.info(self.__class__.__name__ + " succeeded")
            return py_trees.common.Status.SUCCESS
    def terminate(self, new_status):
        self.logger.info(f"terminate: current status: {self.status}, new_status: {new_status}")


def get_tree(ret ="SEQUENCE_ROOT", test_blackboard = False):
    sequence_root = py_trees.composites.Sequence(name = "Test root", memory=True)
    sequence_root.add_children((MoveToStandoffPos(name="move to stand off", test_blackboard=test_blackboard), Grasp(name="Grasp"))) 
    return sequence_root
    
#####################################################
## Test Functions
#####################################################
    
def test_vanilla():
    """
    In a for loop, we will repeat once each behavior is done
        - In a single tick: 
            - Can switch behaviors
        - So we want to use a while loop for status
        - always call tick_once(). each tick() call will the object's tick()
    """
    mtfp = MoveToStandoffPos()
    mtfp.setup()
    for i in range(7):
        mtfp.tick_once()
    mtfp.stop()

def test_sequence():
    '''
    Sequence, Parallel, and Selector are the three composites
    '''
    root = get_tree("SEQUENCE_ROOT")
    root.setup_with_descendants()
    while root.status != py_trees.common.Status.SUCCESS:
        root.tick_once()
        root.logger.warning("status: "+str(root.status))
    root.stop()

def test_decorators():
    def check():
        return True

    foo = py_trees.behaviours.SuccessEveryN(name="foo", n=4)
    root = py_trees.decorators.Condition(child=foo)
    
    root.setup_with_descendants()
    while root.status != py_trees.common.Status.SUCCESS:
        root.tick_once()
        root.logger.warning("status: "+str(root.status))

def test_retry_until_succeed():
    sequence_root = get_tree("SEQUENCE_ROOT")    
    root = py_trees.decorators.FailureIsRunning(
       child = sequence_root 
    )
    root.setup_with_descendants()
    while root.status != py_trees.common.Status.SUCCESS:
        root.tick_once()
        root.logger.warning("status: "+str(root.status))

class FailureIsRunningFiniteTimes(py_trees.decorators.Decorator):
    """
    We need this repeat finite number of times
    See https://answers.ros.org/question/281961/looping-a-sequence-in-py_trees/
    """
    def __init__(self, number_retry: int, child: py_trees.behaviour.Behaviour, name=py_trees.common.Name.AUTO_GENERATED):
        super().__init__(child, name)
        self.NUMBER_RETRY = number_retry

    def initialise(self):
        super().initialise()
        self.retry_count = 0
        
    def update(self):
        """
        Reflect :data:`~py_trees.common.Status.FAILURE` as :data:`~py_trees.common.Status.RUNNING`.

        Returns:
            :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        if self.decorated.status == py_trees.common.Status.FAILURE:
            self.retry_count += 1
            if self.retry_count < self.NUMBER_RETRY:
                return py_trees.common.Status.RUNNING
            else: 
                print("failed count: ", self.retry_count)
                return py_trees.common.Status.FAILURE
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status

def test_retry_finite_number_of_times():
    sequence_root =  py_trees.behaviours.SuccessEveryN(name="foo", n=3)
    root = FailureIsRunningFiniteTimes(
       child = sequence_root, 
       number_retry=2
    )
    root.setup_with_descendants()
    while root.status != py_trees.common.Status.FAILURE:
        root.tick_once()
        root.logger.warning("status: "+str(root.status))

from collections import deque
class GeneratePickUpLocations(py_trees.behaviour.Behaviour):
    '''
    For black board examples, see: behaviours.py
    '''
    def __init__(self, name=""):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(key = PICKUP_LOCATION, access=py_trees.common.Access.WRITE)
        self.location_list = deque([(1,2), (3,4), (5,6)])

    def update(self) -> py_trees.common.Status:
        try:
            val = self.location_list.popleft()
            self.logger.info(f"setting attr: {val}")
            setattr(self.blackboard, PICKUP_LOCATION, val)
            # we are not returning RUNNING because that will cause this state to keep running
            return py_trees.common.Status.SUCCESS
        except IndexError:
            # Returning FAILURE to signal that we are going to start over
            self.logger.info("generation done! ")
            return py_trees.common.Status.FAILURE

def test_parallel():
    # Note that if one child fails, SuccessOnOne doesn't work (it only works with child that that's invalid/success)
    parallel_root = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SuccessOnOne())
    parallel_root.add_children((MoveToStandoffPos(name="move to stand off"), Grasp(name="Grasp"))) 
    parallel_root.setup_with_descendants()
    while parallel_root.status != py_trees.common.Status.FAILURE:
        parallel_root.tick_once()
        parallel_root.logger.warning("status: "+str(parallel_root.status))

def test_blackboard_read_write():
    root = get_tree("SEQUENCE_ROOT", test_blackboard = True)
    behavior_tree = py_trees.trees.BehaviourTree(root=root)
    behavior_tree.setup(timeout = 15)
    # Running this should generate a "KeyError" that the key doesn't exist
    try:
        behavior_tree.tick_tock(
            period_ms=500,
            number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
            pre_tick_handler=None,
            post_tick_handler=None
        )
    except KeyboardInterrupt:
        behavior_tree.interrupt()
    
    
def test_retry_finite_num_with_blackboard():
    sequence_root = py = get_tree("SEQUENCE_ROOT", test_blackboard = True)
    sequence_retry_root = FailureIsRunningFiniteTimes(
       child = sequence_root, 
       number_retry=2
    )
    
    root = py_trees.composites.Sequence()
    root.add_children((GeneratePickUpLocations(), sequence_retry_root))  
    # root.add_children((GeneratePickUpLocations(), sequence_retry_root))  
    # root.setup_with_descendants()
    # while root.status != py_trees.common.Status.FAILURE:
    #     root.tick_once()
    #     root.logger.warning("status: "+str(root.status))

    behavior_tree = py_trees.trees.BehaviourTree(root=root)
    behavior_tree.setup(timeout = 15)
    try:
        behavior_tree.tick_tock(
            period_ms=500,
            number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
            pre_tick_handler=None,
            post_tick_handler=None
        )
    except KeyboardInterrupt:
        behavior_tree.interrupt()


def test_tree():
    root = py_trees.composites.Selector("Selector")
    high = py_trees.behaviours.Success(name="High Priority")
    med = py_trees.behaviours.Success(name="Med Priority")
    low = py_trees.behaviours.Success(name="Low Priority")
    root.add_children([high, med, low])

    behaviour_tree = py_trees.trees.BehaviourTree(
        root=root
    )
    print(py_trees.display.unicode_tree(root=root))
    behaviour_tree.setup(timeout=15)

    def print_tree(tree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))

    def shutdown_tree(button_tree): 
        if button_tree.root.status == py_trees.common.Status.SUCCESS or \
        button_tree.root.status == py_trees.common.Status.FAILURE: 
            button_tree.interrupt_tick_tocking = True
    
    behaviour_tree.add_post_tick_handler(shutdown_tree)
    try:
        behaviour_tree.tick_tock(
            period_ms=500,
            number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
            pre_tick_handler=None,
            post_tick_handler=print_tree
        )
    except KeyboardInterrupt:
        behaviour_tree.interrupt()

SESSION_NAME = 'py_trees_visualization'
import os
def check_and_create_new_tmux_window():
    import subprocess
    command = "byobu-tmux list-windows"
    output = subprocess.check_output(command.split(" ")).decode('utf-8')
    if SESSION_NAME not in output:
        os.system(f'byobu new-window -n {SESSION_NAME}')

def test_tree_printing():
    import functools
    # 1. Adding post tick handler so after the tick, we write the tree to the output file
    def post_tick_handler(snapshot_visitor, behaviour_tree):
        os.system(f"tmux send -t '{SESSION_NAME}' 'clear' ENTER")
        tree_str = py_trees.display.unicode_tree(
            behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.visited
        )
        black_board_str = py_trees.display.unicode_blackboard(
        )
        BEHAVIOUR_TREE_LOG = "/tmp/behavior_tree.log"
        os.system(f"""tmux send-key -R -t '{SESSION_NAME}' 'tail -f {BEHAVIOUR_TREE_LOG}' ENTER""")
        try: 
            with open(BEHAVIOUR_TREE_LOG, "a+") as f:
                f.write(tree_str) 
                f.write("=================================") 
                f.write(black_board_str)
        except: 
            with open(BEHAVIOUR_TREE_LOG, "w") as f:
                f.write(tree_str) 
                f.write("=================================") 
                f.write(black_board_str)

    # Construct the tree
    root = py_trees.composites.Sequence("Sequence")
    for action in ["Action 1", "Action 2", "Action 3"]:
        b = py_trees.behaviours.Count(
                name=action,
                fail_until=0,
                running_until=1,
                success_until=10)
        root.add_child(b)
    root.add_child(GeneratePickUpLocations())
    behaviour_tree = py_trees.trees.BehaviourTree(root)

     # 2. create a new tmux window
    check_and_create_new_tmux_window()
    
    # 3 - bind snapshot visitor and the post tick handler to the tree
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    behaviour_tree.add_post_tick_handler(
        functools.partial(post_tick_handler,
                        snapshot_visitor))
    
    # 4 add the snapshot visitor back to the tree 
    behaviour_tree.visitors.append(snapshot_visitor)
    
    behaviour_tree.tick_tock(
        period_ms=1000,
        number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
        pre_tick_handler=None,
    )


def notes():
    # 1. _generate_text_tree forces to show feedback message if show_status is true, but its status message may NOT be cleared in the most recent tick. So there could be stale feedback messages there. 
    # 2. Py_tree caveat:  `now` is the feedback message of behavior SuccessEveryN

    pass
        
if __name__ == "__main__":
    # test_sequence()
    # test_decorators()
    # test_parallel()
    # test_retry_until_succeed()
    # test_retry_finite_number_of_times()
    test_blackboard_read_write()
    # test_retry_finite_num_with_blackboard()
    # test_tree()
    # test_tree_printing()
