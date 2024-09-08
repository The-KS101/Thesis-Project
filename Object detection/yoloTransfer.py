from ultralytics import YOLO
import torch
import torch.nn as nn

# Define the dataset YAML path
data_yaml = "./datasets/Car-Detection-2/data.yaml"

# Load the pretrained YOLOv8 model
model = YOLO("./Object Detection/yolov8n.pt")

# Modify the model head for new classes
num_new_classes = 27  # Replace with total number of classes
print(model.model.head)
in_features = model.model[-1].in_features  # Number of input features to the final layer
model.model[-1] = nn.Conv2d(in_features, num_new_classes, kernel_size=(1, 1), stride=(1, 1))

# Freeze all layers except the final layer
for param in model.parameters():
    param.requires_grad = False
for param in model.model[-1].parameters():
    param.requires_grad = True

# Train the model with the modified head
model.train(data=data_yaml, epochs=10, imgsz=640)  # Train only the final layer initially

# Unfreeze all layers for fine-tuning
for param in model.parameters():
    param.requires_grad = True

# Continue training the entire model
model.train(data=data_yaml, epochs=1, imgsz=640)  # Fine-tune the entire model

success = model.export(format='onnx')

# Save the fine-tuned model
model.save("yolov8n_custom.pt")
