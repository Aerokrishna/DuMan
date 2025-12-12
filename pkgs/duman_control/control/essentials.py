from google import genai
from google.genai import types
from pydantic import BaseModel, Field
from typing import List
import speech_recognition as sr

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
- Object must be from the provided context, if the object is not present in the cotext return empty list
- In the context the object that the arm is holding is given. If it is not empty, place the object in the parking first then continue with the new command
- When the action is place, the object will be the place object not the object it is holding
- Always return valid JSON only
"""

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

def plan_task(command: str, right_objs, left_objs, right_holding, left_holding) -> Plan:

    client = genai.Client()   # GOOGLE_API_KEY must be exported in environment

    context = (
        f"Human Command: {command}\n"
        f"Right arm can reach: {', '.join(right_objs)}\n"
        f"Left arm can reach: {', '.join(left_objs)}\n"
        f"Left arm is holding: {left_holding}\n"
        f"Right arm is holding: {right_holding}\n"
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

def transcribe_microphone_speech():
 
    r = sr.Recognizer()

    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source, duration=1) 
        print("Say something! (Listening for 5 seconds)")
        
        try:
            audio = r.listen(source, timeout=5, phrase_time_limit=5)
        except sr.WaitTimeoutError:
            print("No speech detected within the time limit.")
            return
    try:
        print("Recognizing speech...")
        text = r.recognize_google(audio)
        
        print("\nTranscription Result:")
        print(f"**You said: \"{text}\"**")
        
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio.")
    except sr.RequestError as e:
        print(f"Could not request results from Google Speech Recognition service; {e}")

if __name__ == "__main__":
    transcribe_microphone_speech()