from google import genai
from google.genai import types
from pydantic import BaseModel, Field
from typing import List

class Step(BaseModel):
    action: str = Field(description="pick | place | pass")
    arm: str = Field(description="right | left")
    obj: str = Field(description="object name")

class Plan(BaseModel):
    steps: List[Step]

# ------------------------------
# 2. System instruction
# ------------------------------
SYSTEM_INSTRUCTION = """
You are a Dual-Arm Manipulator Task Planner.

You MUST output steps as:
{
  "steps": [
      {"action": "...", "arm": "...", "obj": "..."},
      ...
  ]
}

Rules:
- Actions: pick, place, pass
- Arms: right or left
- Object must be from the provided context
- When the action is place, the object will be the place object not the object it is holding
- Always return valid JSON only
"""

def parallelize(plan: Plan):
    parallel_plan = []
    current_group = []

    for step in plan.steps:
        can_parallel = True

        for g in current_group:
            if g.arm == step.arm:
                can_parallel = False
                break
            if g.action == "pass":
                can_parallel = False
                break
            if g.obj == step.obj:
                can_parallel = False
                break

        if can_parallel:
            current_group.append(step)
        else:
            parallel_plan.append(current_group)
            current_group = [step]

    if current_group:
        parallel_plan.append(current_group)

    # Convert to arrays/tuples
    final_array = []
    for group in parallel_plan:
        arr = []
        for s in group:
            arr.append((s.action, s.arm, s.obj))
        final_array.append(arr)

    return final_array

def splitTasks(plan: Plan):
    left_seq = []
    right_seq = []

    for step in plan.steps:
        if step.action == "pass":
            left_seq.append((step.arm, step.action, step.obj))
            right_seq.append((step.arm, step.action, step.obj))
            continue
            
        if step.arm=="left":
            left_seq.append((step.arm, step.action, step.obj))
        else:
            right_seq.append((step.arm, step.action, step.obj))        
    return left_seq, right_seq

def plan_task(command: str, right_objs, left_objs) -> Plan:

    client = genai.Client()   # GOOGLE_API_KEY must be exported in environment

    context = (
        f"Human Command: {command}\n"
        f"Right arm can reach: {', '.join(right_objs)}\n"
        f"Left arm can reach: {', '.join(left_objs)}\n"
        f"Generate the robot plan in JSON ONLY."
    )

    response = client.models.generate_content(
        model="gemini-2.5-flash",
        contents=context,
        config=types.GenerateContentConfig(
            system_instruction=SYSTEM_INSTRUCTION,
            response_mime_type="application/json",
            response_json_schema=Plan.model_json_schema(),
        ),
    )

    return Plan.model_validate_json(response.text)