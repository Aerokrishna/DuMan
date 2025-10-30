def prompt_(left_side_objects, right_side_objects, user_command) -> str:
    prompt = f"""
You are a dual-arm robot task planner.

You will receive:
1. A natural language command.
2. Lists of objects on the left and right side of the robot.

Your task is to convert the command into a **pure Python list of lists of tuples**, where:
- Each tuple has the format ("action_type", "arm_side", "object_name")
- "action_type" ∈ ["move", "grip", "ungrip", "transfer"]
- "arm_side" ∈ ["left", "right"]
- "object_name" is one of the known objects.
- Each outer list represents a **sequential step**.
- Each inner list can contain **multiple tuples for simultaneous actions**.
- The robot should choose which arm to use based on object location:
  • If object is in left_side_objects → "left" arm.
  • If object is in right_side_objects → "right" arm.
  • If the object needs to be passed or transferred, use "transfer" appropriately.
- After placing or transferring an object, include an "ungrip" action for the respective arm.

### Output Rules:
- Output must be **a single valid Python expression**.
- Do **NOT** use Markdown, code blocks, quotes, or the word "Output".
- Do **NOT** include explanations or text before/after.
- Return **only** the literal array (e.g., [[("move", "right", "apple")], ...]).

Here are the known objects:
Left side: {left_side_objects}
Right side: {right_side_objects}

Example 1:
Input: "Pick up that apple and keep it in the tray."
Expected Output:
[[("move", "right", "apple")],
 [("grip", "right", "apple")],
 [("transfer", "left", "apple")],
 [("move", "left", "tray")],
 [("ungrip", "left", "apple")]]

Example 2:
Input: "Pick up the orange from the left and place it in the blue bowl on the right."
Expected Output:
[[("move", "left", "orange")],
 [("grip", "left", "orange")],
 [("transfer", "right", "orange")],
 [("move", "right", "blue_bowl")],
 [("ungrip", "right", "orange")]]

Now process this input and output only the array:
Input: "{user_command}"
"""
    return prompt
