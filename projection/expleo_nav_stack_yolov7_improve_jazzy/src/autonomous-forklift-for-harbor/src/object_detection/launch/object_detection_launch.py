from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    N = 5  # Change this value to the number of nodes you want to launch
    nodes = []

    for i in range(1, N + 1):
        node = Node(
            package='object_detection',
            namespace=f'/home/hdl_sunshine/IlyessOuazzaniChahdi', # Add your namespace
            executable='object_detection_node',
            name=f'Name',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'weight_file_path': '/home/hdl_sunshine/IlyessOuazzaniChahdi/expleo_nav_stack_ws/src/autonomous-forklift-for-harbor/src/object_detection/weight/yolov7.pt', # Add the path to the weights file
                 'conf_thres': 0.25,
                 'iou_thres': 0.45,
                 'device': 'cpu',
                 'img_size': 640
                 }
            ]
        )
        nodes.append(node)

    return LaunchDescription(nodes)

