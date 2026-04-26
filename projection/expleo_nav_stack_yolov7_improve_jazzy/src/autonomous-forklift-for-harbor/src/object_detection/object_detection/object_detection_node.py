# Import required libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import torch
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from perception2d_interfaces.msg import DetectionList, Detection, DetectionBounds, DetectionBoundsList
from cv_bridge import CvBridge
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords, set_logging, check_img_size
from utils.plots import plot_one_box, plot_obstacle
from utils.torch_utils import select_device
from utils.datasets import letterbox
import os
import threading
import time
from scipy.ndimage import uniform_filter1d
# Define the ObjectDetection class that inherits from rclpy's Node class
class ObjectDetection(Node):
    def __init__(self):
        t_init = time.time()
        super().__init__("ObjectDetection")


        #print("Torch version:", torch.__version__)
        #print("CUDA available:", torch.cuda.is_available())
        #print("CUDA device count:", torch.cuda.device_count())
        #for i in range(torch.cuda.device_count()):
            #print(f"Device {i}: {torch.cuda.get_device_name(i)}")

        # Declare and get parameters for the object detection model
        # Declare and get parameters for the object detection model
        self.declare_parameter("weights")
        self.weights = "/home/chaofan/expleo_nav_stack_yolov7_improve_jazzy/src/autonomous-forklift-for-harbor/src/object_detection/weight/yolov7.pt"
        self.declare_parameter("conf_thres")
        self.conf_thres = 0.35
        self.declare_parameter("iou_thres")
        self.iou_thres = 0.45
        self.declare_parameter("device")
        self.device ='0'
        self.declare_parameter("img_size")
        self.img_size = 640


        # Initialize the image and timer
        self.rgb_image = None
        self.depth_image = None

        # Flag that is set to `True` when the `detect()` function is called. 
        # The image processing is now done in a separate thread using the `threading` module. 
        # This allows the main thread to continue receiving new images while the processing is happening in the background. 
        # When the processing is done, the flag is set back to `False`. 
        # This will help prevent skipping frames during image processing and improve overall performance.
        self.processing = False

        # Initialize the CvBridge and subscribers/publishers for images
        self.bridge = CvBridge()
        self.image_rgb_sub = self.create_subscription(Image, '/carla/ego_vehicle/rgb_cam/image', self.image_rgb_callback, 10)
        self.image_depth_sub = self.create_subscription(Image, '/carla/ego_vehicle/depth_cam/image', self.image_depth_callback, 10)
        
        #self.image_sub = self.create_subscription(Image, '/LEFT_image', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, 'YOLO_result', 10)
        self.detection_pub = self.create_publisher(Detection, 'Detection_coordinates', 10)
        self.detection_list_pub = self.create_publisher(DetectionList, 'Detection_coordinates_list', 10)
        self.detection_bounds_pub = self.create_publisher(DetectionBounds, 'Detection_bounds', 10)
        self.detection_bounds_list_pub = self.create_publisher(DetectionBoundsList, 'Detection_bounds_list', 10)
  
        # Set up the object detection model
        set_logging()
        os.environ["CUDA_VISIBLE_DEVICES"] = "0"
        self.device = select_device(self.device)
        self.half = self.device.type != 'cpu'
        self.model = attempt_load(self.weights, map_location=self.device)
        stride = int(self.model.stride.max())
        imgsz = check_img_size(self.img_size, s=stride)
        #if self.half:
        #    self.model.half()

        # Get the model's class names and colors for bounding boxes
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in self.names]
        
        tf_init = time.time()
        dt_init = tf_init - t_init
        print("Durée initialisations :",dt_init)
        
        
    # Callback for receiving images from the rgb camera feed
    def image_rgb_callback(self, data):
        t_callback1 = time.time()
        self.rgb_image = self.bridge.imgmsg_to_cv2(data)
        self.detect()
        tf_callback1 = time.time()
        dt_callback1 = tf_callback1 - t_callback1
        print("Durée callback 1 :",dt_callback1)
            
    # Callback for receiving images from the rgb camera feed
    def image_depth_callback(self, data):
        t_callback2 = time.time()
        self.depth_image = self.bridge.imgmsg_to_cv2(data)
        tf_callback2 = time.time()
        dt_callback2 = tf_callback2 - t_callback2
        print("Durée callback 2 :",dt_callback2)

    # Convert 2D pixel coordinates and depth into 3D spatial coordinates
    def convert_pixel(self, x_rgb, x_depth, label=None):
        if label not in "traffic light":
            i = 0
            x_pixel_list = []
            depth_list = []
            x_list = []
            # Loop through the whole detected obstacles
            while i < len (x_rgb):
                j=0
                while j < 2:   
                    # Extract pixel coordinates of bounding boxes   
                    x_2d_min = (int(x_rgb[i][j]))
                    x_2d_max = (int(x_rgb[i][j+2]))
                    xx_2d_min = (int(x_rgb[i][j+4]))
                    y_2d_min = (int(x_rgb[i][j+1]))
                    y_2d_max = (int(x_rgb[i][j+3]))
                    yy_2d_min = (int(x_rgb[i][j+5]))


                    # Depth of the pixel
                    z_min = x_depth[int(y_2d_min)][int(x_2d_min)]
                    z_max = x_depth[int(y_2d_max)][int(x_2d_max)]
                    zz_min = x_depth[int(yy_2d_min)][int(xx_2d_min)]

                    # We add these values to the created lists
                    x_pixel_list.append(x_2d_min)
                    x_pixel_list.append(x_2d_max)
                    x_pixel_list.append(xx_2d_min)

                    depth_list.append(z_min)
                    depth_list.append(z_max)
                    depth_list.append(zz_min)

                    
                    # Projection/camera matrix
                    # See CameraInfo.msg documentation
                    fx = 854.0449816523632
                    cx = 620.5
                    xx1 = ((x_2d_min - cx) * z_min) / fx
                    xx2 = ((x_2d_max - cx) * z_max) / fx
                    xx_min = ((xx_2d_min - cx) * zz_min) / fx
                    

                    x_list.append(xx1)
                    x_list.append(xx2)
                    x_list.append(xx_min)

                    j+=3
                i +=1
            # Creation of lists to place obstacles around the vehicle
            vehicle_depth_list = []
            vehicle_x_list = []
            
            # The obstacles are moved along x to take account of the fact that the vehicle is positioned at x = 75
            for k in range (0,len(depth_list)) :
                vehicle_depth_list.append(depth_list[k]+2.0)
                vehicle_x_list.append(x_list[k]-0.5)
                       
            detection_list = []
            for i in range (0,len(vehicle_x_list)):
                detection_list.append(vehicle_depth_list[i])
                detection_list.append(vehicle_x_list[i])
            return detection_list
          
    # Identification of the nearest corner in depth data based on gradient changes
    def get_nearest_corner(self, label, distance1, x1, x2, x_min, x_max, threshold=0.25):
        # Extract horizontal slice of depth values between x_min and x_max
        filtered_distance = distance1[x_min - min(x1, x2):x_max - min(x1, x2)]
        if len(filtered_distance) < 2:
            return (x1 + x2) // 2

        # Apply smoothing filter to first derivative to second derivative
        filtered_dist1 = uniform_filter1d(filtered_distance, size=3)
        first_order = [filtered_dist1[i + 1] - filtered_dist1[i] for i in range(len(filtered_dist1) - 1)]
        second_order = [abs(first_order[i + 1] - first_order[i]) for i in range(len(first_order) - 1)]

        #print(f"Distance : {filtered_distance} \n Distance filtered:  {label} {filtered_dist1} \n Dérivée du premier ordre : {first_order} \n Dérivée du second ordre : {second_order}")
            
        # Return center if no variation    
        if not second_order:
            if x_max - x_min > threshold:
                return x_min + (x_max - x_min) // 4 
        # Return point where change is above the threshold
        i_index = second_order.index(max(second_order))
        if second_order[i_index] > threshold:
            return x_min + i_index
        else:
            return x_min + len(second_order) // 2


    # Method for performing object detection on the input image
    def detect(self):
        t_detec = time.time()
        detections = []
        detections_bounds = []
        detection_list = DetectionList()
        detection_bounds_list = DetectionBoundsList()
        filter = ['traffic light','potted plant','umbrella']
        
        # Preprocess the input images
        if self.rgb_image is None or self.depth_image is None: 
            return
        img0_rgb = self.rgb_image.copy()
        img0_depth = self.depth_image.copy()  # Depth image
        img0_rgb = img0_rgb[:, :, :3]  # BGRA to BGR
        img_rgb = letterbox(img0_rgb, self.img_size, auto=False, scaleup=False)[0]
        img_rgb = img_rgb[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, 3x416x416
        img_rgb = torch.from_numpy(img_rgb.copy()).to(self.device).float() / 255.0
        img_rgb = img_rgb.unsqueeze(0)

        # Measure preprocessing duration
        tf_detec = time.time()
        print("Durée init sensors : ", tf_detec - t_detec)
        
        # Inference
        t_model = time.time()
        with torch.no_grad():
            pred = self.model(img_rgb)[0]
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres)
        tf_model = time.time()
        dt_model = tf_model - t_model
        print("Durée du modèle :",dt_model)
        
        # Process detections
        for det in pred:
            if len(det):
                t_detections = time.time()
                det[:, :4] = scale_coords(img_rgb.shape[2:], det[:, :4], img0_rgb.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    if any(filter_word in label for filter_word in filter):                        
                        continue
                    
                    img0_rgb = cv2.UMat(img0_rgb).get()
                    pixel_distances = {}
                    distance1 = []
                    x1, y1, x2, y2 = map(int, xyxy)
                    
                    # Get depth's object of center row
                    depth_crop = img0_depth[int((y1+y2)/2), x1:x2]
                    distance_to_object = np.median(depth_crop)
                    
                    # Visualization of bounding box
                    plot_one_box(xyxy, img0_rgb, label=label, distance=distance_to_object,color=(255, 0, 0), line_thickness=3)
                    
                    # Retrieve bounding box info
                    detection_list0 = plot_obstacle(det[:, :4].tolist(),img0_depth,label=label)
                    k=0
                    
                    # Measure postprocessing duration 
                    tf_detec2 = time.time()
                    dt_detec2 = tf_detec2 - t_detections
                    print("Durée d'execution du bloc :",dt_detec2)
                    
                    # Iterate over each pixel in the bounding box to get the distance  
                    while k < (len(detection_list0)-1):
                        ti_filter = time.time()
                        
                        # Extract all depth values across x-axis center line 
                        for x in range(min(x1, x2), max(x1,x2)):
                            y = int((y1+y2)/2)
                            distance = img0_depth[y, x]
                            distance1.append(distance)  
                            pixel_distances[(x, y)] = distance                            
                        median_dist = np.median(distance1)
                        
                        # Filter out noise in depth
                        threshold_filter = 0.75
                        if abs(median_dist - distance_to_object) < 0.5:
                            x_center = len(distance1)//2
                            
                            # Find left edge
                            for i in range(x_center,1,-1):
                                d1 = distance1[i]
                                d2 = distance1[i-1]
                                if abs(d1 - d2) >= threshold_filter:
                                    break
                            x_min = min(x1,x2)+i
                            #d_min = distance1[i]
                            
                            # Find right edge
                            for i in range(x_center,len(distance1)-1,1):
                                d1 = distance1[i]
                                d2 = distance1[i+1]
                                if abs(d1 - d2) >= threshold_filter:
                                    break
                            #d_max = distance1[i]
                            x_max = min(x1,x2)+i
                            
                            label_class = label.split(" ")[0]                            
                            # Retrieve  the nearest corner                           
                            xx_min = self.get_nearest_corner(label_class, distance1, x1, x2, x_min, x_max)
                            y_min, yy_min, y_max = y, y, y
                            
                            # Convert pixel coordinates of the points in 3d coordinates 
                            bounds_box = []
                            bounds_box.append(x_min)
                            bounds_box.append(y_min)
                            bounds_box.append(x_max)
                            bounds_box.append(y_max)
                            bounds_box.append(xx_min)
                            bounds_box.append(yy_min)
                            detection_bounds0 = self.convert_pixel([bounds_box], img0_depth, label=label)
                            
                            # Measure filter duration
                            tf_convert = time.time()
                            dt_convertfilt = tf_convert - ti_filter
                            print("Durée filtre + conversion pixel : ",dt_convertfilt)

                            # Measures of distance euclidean
                            d1 = np.sqrt((detection_bounds0[0] - detection_bounds0[4])**2 + (detection_bounds0[1] - detection_bounds0[5])**2)
                            d2 = np.sqrt((detection_bounds0[2] - detection_bounds0[4])**2 + (detection_bounds0[3] - detection_bounds0[5])**2)
                            threshold = 0.5
                            
                            
                            # If distance is too low, discard middle point
                            if d1 < threshold or d2 < threshold:
                                detection_bounds0[len(detection_bounds0)*k+4] = 0.0
                                detection_bounds0[len(detection_bounds0)*k+5] = 0.0
                                
                            # Add the detection to the DetectionBoundsList message
                            detection_bounds = DetectionBounds()
                            detection_bounds.x_min = detection_bounds0[len(detection_bounds0)*k]
                            detection_bounds.y_min = detection_bounds0[len(detection_bounds0)*k+1]
                            detection_bounds.x_max = detection_bounds0[len(detection_bounds0)*k+2]
                            detection_bounds.y_max = detection_bounds0[len(detection_bounds0)*k+3]
                            detection_bounds.xx_min = detection_bounds0[len(detection_bounds0)*k+4]
                            detection_bounds.yy_min = detection_bounds0[len(detection_bounds0)*k+5]
                            detection_bounds.label = label_class
                            detections_bounds.append(detection_bounds)

                            detection_list0 = plot_obstacle([[x_min,y_min,x_max,y_max]],img0_depth,label=label)                    
                            # Add the detection to the DetectionList message
                            detection_coord = Detection()
                            detection_coord.x = detection_list0[k]
                            detection_coord.y = detection_list0[k+1]
                            detections.append(detection_coord)  
                        
                        k+=2
                        tf_detections = time.time()
                        dt_detections = tf_detections - t_detections
                        print("Durée bounding box + coord",dt_detections)
        # Populate and publish DetectionList
        t_ros = time.time()
        detection_list.detection_list = detections
        self.detection_list_pub.publish(detection_list)
        
        # Populate and publish DetectionBoundsList
        detection_bounds_list.detection_bounds_list = detections_bounds
        self.detection_bounds_list_pub.publish(detection_bounds_list)
        # Convert processed image to ROS message and publish
        img_rgb = img0_rgb.astype(np.uint8)
        img_msg = self.bridge.cv2_to_imgmsg(img_rgb, encoding="bgr8")
        self.image_pub.publish(img_msg)
        # End processing
        self.processing = False
        print(f"Nombre de points transférés : {len(detections)}")
        tf_detec = time.time()
        dt_ros = tf_detec - t_ros
        print("Durée message ROS",dt_ros)
        dt_detec = tf_detec - t_detec
        print("Durée detection :",dt_detec)

        
# Main function for running the object detection node
def main(args=None):
    rclpy.init(args=args)
    with torch.no_grad():
        object_detection_node = ObjectDetection()
        rclpy.spin(object_detection_node)
