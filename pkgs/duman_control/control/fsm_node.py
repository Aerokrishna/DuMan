#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Blackboard:
    """Shared memory for inter-state communication."""
    def __init__(self):
        self.data = {}

    def set(self, key, value):
        self.data[key] = value

    def get(self, key, default=None):
        return self.data.get(key, default)


class FSMNode(Node):
    """Finite State Machine integrated as a ROS 2 node."""

    def __init__(self):
        super().__init__('fsm_node')

        # Shared blackboard for inter-state flags
        self.bb = Blackboard()

        # Define FSM structure { state_name: (function, completion_condition, next_state) }
        self.states = {
            "SCAN": (self.state_scan, lambda bb: bb.get("scan_done", False), "MOVE"),
            "MOVE": (self.state_move, lambda bb: bb.get("move_done", False), "PICK"),
            "PICK": (self.state_pick, lambda bb: bb.get("pick_done", False), None),
        }

        # Start FSM
        self.current_state = None
        self.timer = None
        self.change_state("SCAN")

    # ---------------------------
    # Core FSM Logic
    # ---------------------------
    def change_state(self, next_state_name):
        """Transition to the given state and start its execution timer."""
        if self.timer:
            self.timer.cancel()  # stop the previous timer

        if next_state_name is None:
            self.get_logger().info("[FSM] Final state reached. Stopping node.")
            return

        self.current_state = next_state_name
        self.get_logger().info(f"[FSM] Entering state: {self.current_state}")

        # Retrieve the state function, condition, and next state
        state_fn, condition_fn, next_state = self.states[self.current_state]

        # Run the state function periodically
        self.timer = self.create_timer(0.5, lambda: self.state_step(state_fn, condition_fn, next_state))

    def state_step(self, fn, cond, next_state):
        """Execute state and check for completion condition."""
        fn(self.bb)  # run the state function

        if cond(self.bb):  # check condition
            self.get_logger().info(f"[FSM] Transition: {self.current_state} → {next_state}")
            self.change_state(next_state)

    # ---------------------------
    # State Functions
    # ---------------------------
    def state_scan(self, bb: Blackboard):
        self.get_logger().info("Scanning environment...")
        bb.set("scan_done", True)

    def state_move(self, bb: Blackboard):
        moved = bb.get("moved", 0)
        self.get_logger().info(f"Moving to target... step {moved+1}")
        if moved >= 3:
            bb.set("move_done", True)
        else:
            bb.set("moved", moved + 1)

    def state_pick(self, bb: Blackboard):
        self.get_logger().info("Picking object...")
        bb.set("pick_done", True)


# ---------------------------
# Main Entry Point
# ---------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down FSM node.")
    finally:
        if node.timer:
            node.timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
