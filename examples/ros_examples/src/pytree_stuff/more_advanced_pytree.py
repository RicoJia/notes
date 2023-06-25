#!/usr/bin/env python3
import py_trees
from py_trees.common import Status

"""
Tree structure: 
selector: node 2, node 3(SIR)
    - node 2: fail, succeed
    - Node 3: Running, succeed
"""
SUCCESS = Status.SUCCESS
FAILURE = Status.FAILURE
RUNNING = Status.RUNNING

def test_interesting_tree():
    # 1. Selector: if node 2 is RUNNING, next tick it will still be node 2. Once node 2 fails, in the same tick it will 
    # transition to node 3. 
    # 2. But memory is False, every tick will start from the beginning. 
    priorities = py_trees.composites.Selector(name="Tasks", memory=False)
    board = py_trees.behaviours.StatusQueue(name="board", queue=[RUNNING, FAILURE], eventually=SUCCESS)
    # Eternal Guard first examine condition. If it's good, then tick
    ready_guard = py_trees.decorators.EternalGuard(
        name="Ready Guard",
        condition=lambda: SUCCESS,
        child=board
    )
    approach = py_trees.behaviours.StatusQueue(name="approach", queue=[RUNNING, RUNNING, RUNNING, FAILURE, SUCCESS], eventually=SUCCESS)
    approachSIR = py_trees.decorators.SuccessIsRunning(
        name="Approach SIR",
        child=approach)

    priorities.add_children([ready_guard, approachSIR])
    root = priorities
    root.setup_with_descendants()
    while root.status != py_trees.common.Status.SUCCESS:
        root.tick_once()
        root.logger.warning("status: "+str(root.status))
    root.stop()

# Notes about BT:
# When elevator boarding fails, go to approach waypoint. But the condition will check if the best_ready_elevator is anticipated. If not, it will fail immediately and 
# go to await_observe. memory = False will check from the first node, while memory = True will check from the current node. So, every time, it will jump back to 
# the first node

# Below example shows how to: 
#   1. set blackboard variables with namespaces 
#   2. see the entire blackboard storage, regardless of namespace
#   3. retrieve shared blackboard variables in another behaviour
PRIVATE_KEY="PRIVATE_KEY"
PUBLIC_KEY="PUBLIC_KEY"
def test_blackboard_ns():
    class Behavior1(py_trees.behaviour.Behaviour):
        def __init__(self, name=""):
            super().__init__(name)
            # Example 1 - Have a private blackboard and a public blackboard
            self.public_blackboard = self.attach_blackboard_client(name = "public_bb", namespace = self.__class__.__name__)
            self.public_blackboard.register_key(key = PUBLIC_KEY, access=py_trees.common.Access.WRITE)
            self.private_blackboard = self.attach_blackboard_client(name = "private_bb")
            self.private_blackboard.register_key(key = PRIVATE_KEY, access=py_trees.common.Access.WRITE)
        def update(self) -> py_trees.common.Status:
            # Example 2 - Set Private and Public BB Variables
            self.private_blackboard.PRIVATE_KEY = self.__class__.__name__
            self.public_blackboard.PUBLIC_KEY = "public variable"
            # can see blackboard storage: blackboard storage: {'/PRIVATE_KEY': 'Behavior1', '/Behavior1/PUBLIC_KEY': 'public variable'}
            print(f'blackboard storage: {py_trees.blackboard.Blackboard.storage}')
            return py_trees.common.Status.SUCCESS

    class Behavior2(Behavior1):
        def update(self) -> py_trees.common.Status:
            # blackboard storage: {'/PRIVATE_KEY': 'Behavior2', '/Behavior1/PUBLIC_KEY': 'public variable', '/Behavior2/PUBLIC_KEY': 'public variable'}
            super().update()
            # Example 3: Set Public Variables:
            self.public_blackboard.PUBLIC_KEY = "public variable 2"
            # Blackboard storage: {'/PRIVATE_KEY': 'Behavior2', '/Behavior1/PUBLIC_KEY': 'public variable', '/Behavior2/PUBLIC_KEY': 'public variable 2'}
            print(f'blackboard storage: {py_trees.blackboard.Blackboard.storage}')
            return py_trees.common.Status.SUCCESS

    # When ticking, all nodes leading to the leaf node can annotate tree root
    root = py_trees.composites.Sequence(name="Tasks", memory=False)
    root.add_children([Behavior1(), Behavior2()])
    behaviour_tree = py_trees.trees.BehaviourTree(
        root=root
        )
    behaviour_tree.setup(timeout=15)
    while root.status != py_trees.common.Status.SUCCESS:
        root.tick_once()
        root.logger.info("status: "+str(root.status))
    root.stop()

def binding_initialise_with_get():
    """
    orig_on_enter = leaf_node.initialise
    # Tree doesn't have name, only node does
    tree_name = self.tree.root.name
    def new_initialize(self):
        FailureLogging.init_bt_failure_logging(tree_name)
        orig_on_enter()
    # __get__(): descriptor. this binds a function with a class
    leaf_node.initialise = new_initialize.__get__(leaf_node)
    """

if __name__ == "__main__":
    # test_interesting_tree()
    test_blackboard_ns()
