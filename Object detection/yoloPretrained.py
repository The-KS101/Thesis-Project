from ultralytics import YOLO

# Load pretrained best.pt model (Our initially trained model missing potholes)
model = YOLO('./runs/detect/train4/weights/best.pt')

# Train the model using the 'Car-detection-2/data.yaml' dataset for 20 epochs
results = model.train(data='./datasets/Car-Detection-2/data.yaml' epochs=20)

# Evaluate the model's performance on the validation set
results = model.val()

# Export the model to ONNX format
success = model.export(format='onnx')

# Perform object detection on an image using the model, provide file names to run model on them
results = model.predict(["lagos-and-potholes-2.jpeg", "Potholes.jpg", "Potholes (1).jpg"], save=True, imgsz=640, conf=0.5, stream=True)

# Process results list
for result in results:
    boxes = result.boxes  
    masks = result.masks  
    keypoints = result.keypoints
    probs = result.probs 
    result.show()  
    result.save(filename='result.jpg')
