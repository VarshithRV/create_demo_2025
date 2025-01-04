from ultralytics import SAM
import cv2
import numpy as np

pt1 = [579, 276]
pt2 = [1062,738]
dimension = [pt2[0]-pt1[0],pt2[1]-pt1[1]]

# draw the bounding box on the input image
image = cv2.imread("assets/test_image_1.jpeg")
cv2.rectangle(image, (pt1[0], pt1[1]), (pt2[0], pt2[1]), (0, 255, 0), 2)

# Load a model
model = SAM("sam_b.pt")

# Display model information (optional)
model.info()

# Run inference with bboxes prompt
results = model("assets/test_image_1.jpeg", bboxes=pt1.append(pt2), labels=[1])

del model
# plot the image with segmentation and save it
# cv2.imwrite("inference_images/segmented_image.jpg", results[0])

print(type(results[0]))
print("##########################")
print(results[0])
print("##########################")


# type(results)
# print(results)

# # Run inference with single point
# results = model(points=[900, 370], labels=[1])

# # Run inference with multiple points
# results = model(points=[[400, 370], [900, 370]], labels=[1, 1])

# # Run inference with multiple points prompt per object
# results = model(points=[[[400, 370], [900, 370]]], labels=[[1, 1]])

# # Run inference with negative points prompt
# results = model(points=[[[400, 370], [900, 370]]], labels=[[1, 0]])