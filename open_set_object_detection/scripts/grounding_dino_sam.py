from groundingdino.util.inference import load_model, load_image, predict, annotate
import cv2
import os
import torch
from segment_anything import sam_model_registry, SamPredictor
import time


model = load_model("groundingdino/config/GroundingDINO_SwinT_OGC.py", "weights/groundingdino_swint_ogc.pth")
IMAGE_PATH = "assets/test_image_1.jpeg"
TEXT_PROMPT = "shapes"
BOX_TRESHOLD = 0.35
TEXT_TRESHOLD = 0.25

# image_cpu = cv2.imread(IMAGE_PATH)

image_source, image = load_image(IMAGE_PATH)

boxes, logits, phrases = predict(
    model=model,
    image=image,
    caption=TEXT_PROMPT,
    box_threshold=BOX_TRESHOLD,
    text_threshold=TEXT_TRESHOLD
)

annotated_frame = annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)

print("########################")
print("\nBoxes : ")
print(boxes, type(boxes))
print("\n")

print("########################")
print("\nLogits : ")
print(logits, type(logits))
print("\n")

print("########################")
print("\nPhrases : ")
print(phrases, type(phrases))
print("\n")

print("########################")
print("\nShape of the image : ")
print(image_source.shape)
print("\n")

cv2.imwrite("inference_images/annotated_image.jpg", annotated_frame)

print("memory used : ", torch.cuda.memory_allocated(device="cuda")/1024/1024)

# delete all variables
del model, IMAGE_PATH, TEXT_PROMPT, BOX_TRESHOLD, TEXT_TRESHOLD, image_source, boxes, logits, phrases

sam_checkpoint = "sam_vit_h_4b8939.pth"
model_type = "vit_h"
device = "cuda"

print("memory used : ", torch.cuda.memory_allocated(device=device)/1024/1024)

sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
sam.to(device=device)
predictor = SamPredictor(sam)

print("memory used : ", torch.cuda.memory_allocated(device=device)/1024/1024)

# segmentation
predictor.set_image(image)

print("memory used : ", torch.cuda.memory_allocated(device=device)/1024/1024)