import functools
import os
import subprocess
import py_trees
class TreeVisualizer:
    def __init__(self, log_path = "/tmp/behavior_tree.log", session_name = "py_trees_visualization"):
        self.log_path = log_path
        self.session_name = session_name

    def add_visualizer_to_tree(self, behaviour_tree):
        # bind snapshot visitor and the post tick handler to the tree
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        behaviour_tree.add_post_tick_handler(
        functools.partial(self._post_tick_handler,
                        snapshot_visitor))
        # create a new tmux window
        self._check_and_create_new_tmux_window()
        # 4 add the snapshot visitor back to the tree 
        behaviour_tree.visitors.append(snapshot_visitor)

    def _check_and_create_new_tmux_window(self):
        command = "byobu-tmux list-windows"
        output = subprocess.check_output(command.split(" ")).decode('utf-8')
        if self.session_name not in output:
            os.system(f'byobu new-window -n {self.session_name}')

    def _post_tick_handler(self, snapshot_visitor, behaviour_tree):
        # post tick handler to be added.
        # so after the tick, we write the tree to the output file and visualize it
        os.system(f"tmux send -t '{self.session_name}' 'clear' ENTER")
        tree_str = py_trees.display.unicode_tree(
            behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.visited
        )
        black_board_str = py_trees.display.unicode_blackboard(
        )
        os.system(f"""tmux send-key -R -t '{self.session_name}' 'tail -f {self.log_path}' ENTER""")
        with open(self.log_path, "w") as f:
            f.write(tree_str) 
            f.write("================================= ") 
            f.write(black_board_str)

if __name__ == "__main__":
    # Construct the tree
    from vanila_rjje_arm_pytree import GeneratePickUpLocations
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

    # User code
    tv = TreeVisualizer()
    tv.add_visualizer_to_tree(behaviour_tree)
    behaviour_tree.tick_tock(
        period_ms=1000,
        number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
        pre_tick_handler=None,
    )