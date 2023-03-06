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

if __name__ == "__main__":
    test_interesting_tree()
