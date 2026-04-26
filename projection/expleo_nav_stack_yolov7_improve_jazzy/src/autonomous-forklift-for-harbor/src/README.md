# ROS2 Object Detection Package

This package provides a ROS2 node for object detection in images using a YOLOv7 model. The object detection node subscribes to an image topic, processes the images to detect objects, and publishes the results with bounding boxes and confidence scores.

### Main Components

- `launch/object_detection_launch.py`: Launch file to start multiple instances of the object detection node.
- `models/`: Contains the YOLOv7 model implementation.
- `object_detection/object_detection_node.py`: Main ROS2 node for object detection.
- `utils/`: Contains various utilities for data processing and model-related operations.
- `weights/best.pt`: Pre-trained YOLOv7 model weights file.

## Dependencies

- ROS2 Foxy or newer
- Python 3.6 or newer
- OpenCV
- PyTorch
- numpy
- cv_bridge

## Usage

1. Clone the package into your ROS2 workspace.
2. Build the workspace using `colcon build`.
3. Source the workspace: `source install/setup.bash`.
4. To launch a single object detection node, run: 
`ros2 run object_detection object_detection_node`
5. To launch multiple instances of the object detection node, update the `N` variable in `launch/object_detection_launch.py` and run:

`ros2 launch object_detection object_detection_launch.py`


## Configuration

The object detection node uses several parameters, which can be set in the launch file or by updating the `object_detection_node.py` file:

- `weight_file_path`: Path to the pre-trained YOLOv7 model weights file.
- `conf_thres`: Confidence threshold for object detection.
- `iou_thres`: Intersection over union (IoU) threshold for non-maximum suppression.
- `device`: Device to run the object detection model on (e.g., 'gpu' or 'cpu').
- `img_size`: Image size for the object detection model.

Please ensure the specified pre-trained model weights file is present in the `weights` directory before launching the nodes.

## Note
code de Imad El-bouazzaoui 2023

