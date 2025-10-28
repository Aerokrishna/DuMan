from google import genai
import os
import json

# export GOOGLE_API_KEY="YOUR_KEY"
client = genai.Client(api_key="APIKEY")

user_command = "Pick up that banana and keep it in the box."

prompt = f"""
You are a dual-arm robot task planner.

You will receive a natural language instruction about manipulating objects using a dual-arm robot.
Your job is to convert the instruction into a structured Python list of lists of tuples, where:
- Each tuple has the format ("action_type", "arm_side", "object_name")
- "action_type" ∈ ["move", "grip", "transfer"]
- "arm_side" ∈ ["left", "right"]
- "object_name" is the target (like "apple", "tray", etc.)
- Each *outer list* represents a sequential step
- Each *inner list* can have one or more tuples that can be executed simultaneously (parallel actions)
- Do NOT include any extra text, explanation, JSON, or Markdown. Output ONLY the Python structure.

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

Now, process the following instruction and output only the structure:
Input: "{user_command}"
"""

response = client.models.generate_content(
    model="gemini-2.5-pro",
    contents=[prompt],
)

print("\n--- LLM Output ---")
print(response.text)

try:
    plan = eval(response.text)
    print("\n--- Parsed Plan ---")
    print(plan)
except Exception as e:
    print("\nCould not parse output, raw text:")
    print(response.text)

