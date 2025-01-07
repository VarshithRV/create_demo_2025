import openai
import json
import numpy as np
import cv2

# Initialize OpenAI client
client = openai.Client()
preamble = "You are a robot controller, you can choose the objects you can manipulate you can only do one action \"pick\", you need to output in the following way, \"[<object_id_1>,<object_id_2> ...]\", for picking up <object_id_1> and <object_id_2>. If the robot needs to take any action based on the prompt, use the pick action."

def call_gpt4_with_annotations(user_prompt, object_detections):
    object_detections = json.dumps(object_detections, indent=2)
    
    completion = client.chat.completions.create(
        model="gpt-4o-mini",  # Assuming GPT-4 with vision is available
        messages=[
            {"role": "system", "content": preamble},
            {"role": "user", "content": f"User Prompt: {user_prompt}\nImage Annotations: {object_detections}"}
        ],
        max_tokens=150,
        temperature=0
    )

    # Return the generated response
    pick_list = json.loads(completion.choices[0].message.content)
    for object in pick_list :
        object = int(object)
        
    return pick_list


def main():
    
    image_annotations = {
        "objects": [
            {"id": 0, "label": "red _ triangle"},
            {"id": 1, "label": "green _ rectangle"},
            {"id": 2, "label": "blue _ circle"},
            {"id": 3, "label": "green _ circle"}
        ]
    }
    
    # Step 2: Define the user prompt
    user_prompt = "Pick up the red triangle and the circle and place it inside the cup."

    # Step 3: Call GPT-4 with the user prompt and JSONified image annotations
    print("\nCalling GPT-4 API...")
    gpt4_response = call_gpt4_with_annotations(image_annotations, user_prompt)
    
    print("\nGPT-4 Response: ", gpt4_response, type(gpt4_response), type(gpt4_response[0]))


if __name__ == "__main__":
    main()
