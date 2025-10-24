#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class State:
    """Represents a single FSM state."""
    def __init__(self, name, action_fn, condition_fn):
        self.name = name
        self.action = action_fn
        self.condition = condition_fn
        self.data = {}  # local memory per state

class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')

        # Define states in sequential order
        self.states = [
            State("SCAN", self.scan, lambda s: s.data.get("done", False)),
            State("MOVE", self.move, lambda s: s.data.get("done", False)),
            State("PICK", self.pick, lambda s: s.data.get("done", False)),
        ]

        self.index = 0
        self.current = self.states[self.index]

        self.get_logger().info(f"[FSM] Starting at state: {self.current.name}")
        self.timer = self.create_timer(0.5, self.step)

    def step(self):
        """Run the current state's behavior and transition if done."""
        self.current.action(self.current)

        if self.current.condition(self.current):
            if self.index + 1 >= len(self.states):
                self.get_logger().info("[FSM] Final state reached. Stopping node.")
                self.timer.cancel()
                return

            self.index += 1
            self.current = self.states[self.index]
            self.previous = None
            self.get_logger().info(f"[FSM] Transition → {self.current.name}")

    # ---------------------------
    # State Behaviors
    # ---------------------------
    def scan(self, state):
        self.get_logger().info("Scanning environment...")
        state.data["done"] = True

    def move(self, state):
        step = state.data.get("step", 0)
        self.get_logger().info(f"Moving to target... step {step+1}")
        if step >= 3:
            state.data["done"] = True
        else:
            state.data["step"] = step + 1

    def pick(self, state):
        self.get_logger().info("Picking object...")
        state.data["done"] = True

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
        node.timer.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
