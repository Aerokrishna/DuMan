from google import genai
import os
import json

client = genai.Client(api_key="API")

user_command = "keep the apple in the tray and banana in the cup"

# from the camera
left_side_objects = ["tray", "banana"]
right_side_objects = ["apple", "orange", "cup"]

# Prompt for Gemini
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
- The output must be a valid Python literal (list of lists of tuples).
- Do NOT include any explanation, Markdown, or comments. Output ONLY the structure.

Here are the known objects:
Left side: {left_side_objects}
Right side: {right_side_objects}

Example 1:
Input: "Pick up that apple and keep it in the tray."
Output:
[[("move", "right", "apple")],
 [("grip", "right", "apple")],
 [("transfer", "left", "apple")],
 [("move", "left", "tray")],
 [("grip", "left", "tray")]]

Example 2:
Input: "Pick up the orange from the left and place it in the blue bowl on the right."
Output:
[[("move", "left", "orange")],
 [("grip", "left", "orange")],
 [("transfer", "right", "orange")],
 [("move", "right", "blue_bowl")],
 [("grip", "right", "blue_bowl")]]

Now process the following input and output only the Python structure:
Input: "{user_command}"
"""

response = client.models.generate_content(
    model="gemini-2.5-pro",
    contents=[prompt],
)

print("\n--- LLM Output ---")
print(response.text)

# Safely evaluate the structured output if valid
try:
    plan = eval(response.text)
    print("\n--- Parsed Plan ---")
    print(plan)
except Exception as e:
    print("\nCould not parse output, raw text:")
    print(response.text)
