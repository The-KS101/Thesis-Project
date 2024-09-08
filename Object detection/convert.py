import json
import os

# Convert COCO annotation to YOLO Annotation.

def coco_to_yolo(coco_json_path, output_dir):
    # Load the COCO annotations file
    with open(coco_json_path, 'r') as f:
        coco_data = json.load(f)
    
    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Dictionary to map image id to file name
    image_id_to_filename = {image['id']: image['file_name'] for image in coco_data['images']}
    image_id_to_size = {image['id']: (image['width'], image['height']) for image in coco_data['images']}
    
    # Process each annotation
    for annotation in coco_data['annotations']:
        image_id = annotation['image_id']
        category_id = annotation['category_id']
        bbox = annotation['bbox']
        
        x_min, y_min, width, height = bbox
        image_width, image_height = image_id_to_size[image_id]
        
        # Convert to YOLO format
        x_center = (x_min + width / 2) / image_width
        y_center = (y_min + height / 2) / image_height
        norm_width = width / image_width
        norm_height = height / image_height
        
        # YOLO annotation line
        yolo_annotation = f"{category_id} {x_center} {y_center} {norm_width} {norm_height}\n"
        
        # Get the corresponding image file name
        image_filename = image_id_to_filename[image_id]
        label_filename = os.path.splitext(image_filename)[0] + ".txt"
        label_filepath = os.path.join(output_dir, label_filename)
        
        with open(label_filepath, 'a') as label_file:
            label_file.write(yolo_annotation)

coco_json_path = './potholes/valid/_annotations.coco.json'
output_dir = './valid/labels'
coco_to_yolo(coco_json_path, output_dir)
