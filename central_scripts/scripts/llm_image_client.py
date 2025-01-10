from openai import OpenAI
import json
import numpy as np
import cv2
import base64

client = OpenAI()

# Function to encode the image
def encode_image(image):
    buffer = cv2.imencode('.png', image)[1]
    image_base64 = base64.b64encode(buffer).decode('utf-8')
    print("Image encoded")
    return image_base64


# Path to your image
image_path = "./object_image.png"
image = cv2.imread(image_path)

# Getting the base64 string
base64_image = encode_image(image)

preamble = "You are a robot controller, you can choose the objects you can manipulate you can only do one action \"pick\", you need to output in the following way, \"[<object_id_1>,<object_id_2> ...]\", for picking up <object_id_1> and <object_id_2>. Note that the output should be in this format and should not contain anything else like explanation. If the robot needs to take any action based on the prompt, use the pick action. Refer the image and use it for spatial reasoning if necessary"


image_annotations = {
        "objects": [
            {"id": 0, "label": "red _ triangle"},
            {"id": 1, "label": "green _ rectangle"},
            {"id": 2, "label": "blue _ circle"},
            {"id": 3, "label": "green _ circle"}
        ]
    }

prompt = input("Enter the prompt: ")

print("requesting llm")
completion = client.chat.completions.create(
            model="gpt-4o", 
            messages=[
                {
                    "type" : "text",
                    "role": "system", 
                    "content": preamble
                },
                {
                    "type" : "text",
                    "role": "user", 
                    "content": f"User Prompt: {prompt}\nImage Annotations: {image_annotations}"
                },
                {
                    "type": "image_url",
                    "role" : "user",
                    "content" : "This is the image",
                    "image_url" : {"url": f"data:image/jpg;base64,{base64_image}"}
                }

            ],
            max_tokens=150,
            temperature=0
        )

print(completion.choices[0])
pick_list = json.loads(completion.choices[0].message.content)
for object in pick_list:
    object = int(object)
print(f"The llm returned with object list : {pick_list}, {type(pick_list[0])}")