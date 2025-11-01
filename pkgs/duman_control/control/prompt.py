def prompt_(left_side_objects, right_side_objects, user_command) -> str:
    prompt = f"""
You are a dual-arm robot task planner.

You will receive:
1. A natural language command.
2. Lists of objects on the left and right side of the robot.

You must convert the command into a Python list of lists of tuples, where:
- Each tuple has the format ("action_type", "arm_side", "object_name")
- "action_type" ∈ ["move", "grip", "transfer"]
- "arm_side" ∈ ["left", "right"]
- "object_name" is one of the known objects.
- Each outer list is a step in sequence.
- Each inner list can contain multiple tuples for simultaneous actions.
- The robot should choose which arm to use based on object location:
  • If object is in left_side_objects → "left" arm.
  • If object is in right_side_objects → "right" arm.
  • If the object needs to be passed or transferred, use "transfer" appropriately.
- the robot cannot transfer transfer two objects from both the arms at once

- Output must be a single valid Python expression.
- Do **NOT** include Markdown, code blocks, quotes, or explanations.
- Return only the literal nested list, e.g.:
  [[("move", "right", "apple")], [("grip", "right", "apple")], ...]

Here are the known objects:
Left side: {left_side_objects}
Right side: {right_side_objects}

Example 1:
Input: "Keep the apple in the tray and banana in the cup"
Expected Output:
[[("move", "right", "apple")],
 [("grip", "right", "apple")],
 [("transfer", "left", "apple")],
 [("move", "left", "tray"), ("move", "right", "banana")],
 [("ungrip", "left", "apple"), ("grip", "right", "banana")],
 [("transfer", "left", "banana")],
 [("move", "left", "cup")],
 [("ungrip", "left", "banana")]]

Example 2:
Input: "Pick up that apple and keep it in the tray."
Expected Output:
[[("move", "right", "apple")],
 [("grip", "right", "apple")],
 [("transfer", "left", "apple")],
 [("move", "left", "tray")],
 [("ungrip", "left", "apple")]]

Now process this command and output only the array:
Input: "{user_command}"
"""
    return prompt
