# Carla Ros2 publisher : GNSS OK, IMU OK, ODOMETY ok, IMAGES ok
import carla
import rclpy
import random
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix,Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header
import cv2
import numpy as np
from cv_bridge import CvBridge
import logging

import argparse




def place_ego_vehicle(world):
    ego_spawn_location = carla.Location(x=140, y=237  , z=0.5)
    ego_transform = carla.Transform(ego_spawn_location, carla.Rotation(0,180,0))
    ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    ego_bp.set_attribute('role_name', 'ego')
    ego_vehicle = world.try_spawn_actor(ego_bp, ego_transform)
    if ego_vehicle is not None:
        ego_vehicle.set_autopilot(True) 
        return ego_vehicle
    print("Failed to spawn the ego vehicle")
    return None

class CarlaROSPublisher(Node):
    def __init__(self, args):
        super().__init__('CarlaROSPublisher')
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        client.set_timeout(10.0) 
        world = client.load_world('Town02')
        # --------------
        # Set Synchronous mode
        #--------------
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0
        world.apply_settings(settings)
        # --------------
        # Place the ego vehicle
        # --------------
        ego_vehicle = place_ego_vehicle(world)

        # --------------
        # publisher init
        # --------------
        self.GNSS_publisher = self.create_publisher(NavSatFix, '/GNSS', 10)
        self.IMU_publisher = self.create_publisher(Imu, '/IMU', 10)
        self.ODOMETRY_publisher = self.create_publisher(Odometry, '/ODOMETRY', 10)
        self.LEFT_IMAGE_publisher = self.create_publisher(Image, '/LEFT_image', 10)
        self.RIGHT_IMAGE_publisher = self.create_publisher(Image, '/RIGHT_image', 10)
        self.bridge = CvBridge()
        
        # ------------------------------------------
        # Add vehicles     
        # ------------------------------------------
        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = world.get_blueprint_library()


        transform = random.choice(world.get_map().get_spawn_points())
        transform.location += carla.Location(x=40, y=-3.2)
        transform.rotation.yaw = -180.0
        actor_list = []
        for _ in range(0, args.number_of_vehicles):
            transform.location.x += 8.0

            bp = random.choice(blueprint_library.filter('vehicle'))
	    
            # This time we are using try_spawn_actor. If the spot is already
            # occupied by another object, the function will return None.
            npc = world.try_spawn_actor(bp, transform)
            if npc is not None:
                actor_list.append(npc)
                npc.set_autopilot(True)
                print('created %s' % npc.type_id)

        #time.sleep(5)
        # ------------------------------------------
        # Add walkers   
        # ------------------------------------------
        # The world contains the list blueprints that we can use for adding new
        SpawnActor = carla.command.SpawnActor
        walkers_list = []
        all_id =[]
        blueprintsWalkers = blueprint_library.filter('walker.pedestrian.*')
	
        percentagePedestriansRunning = args.percentagePedestriansRunning     # how many pedestrians will run
        percentagePedestriansCrossing = args.percentagePedestriansCrossing     # how many pedestrians will walk through the road
     
        world.set_pedestrians_seed(args.seedw)
        random.seed(args.seedw)
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(args.number_of_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # 4. we put together the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)
        
        # ------------------------------------------
        # Add a RGB stereo camera sensor     
        # ------------------------------------------
        width = 1241
        height = 376
        fov=72
        
        cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        cam_bp.set_attribute("fov",str(fov))
        cam_bp.set_attribute("image_size_x",str(width))
        cam_bp.set_attribute("image_size_y",str(height))
        cam_bp.set_attribute('enable_postprocess_effects', 'True')
        cam_bp.set_attribute('sensor_tick', '0.1')
        cam_bp.set_attribute('gamma', '2.2')
        cam_bp.set_attribute('motion_blur_intensity', '0')
        cam_bp.set_attribute('motion_blur_max_distortion', '0')
        cam_bp.set_attribute('motion_blur_min_object_screen_size', '0')
        cam_bp.set_attribute('shutter_speed', '1000') #1 ms shutter_speed
        cam_bp.set_attribute('lens_k', '0')
        cam_bp.set_attribute('lens_kcube', '0')
        cam_bp.set_attribute('lens_x_size', '0')
        cam_bp.set_attribute('lens_y_size', '0')
        left_cam_transform = carla.Transform(carla.Location(x=2, y=0, z=1.7), carla.Rotation(pitch=0.0, yaw=0.0, roll=0))
        left_cam = world.spawn_actor(cam_bp, left_cam_transform,attach_to=ego_vehicle)
        right_cam_transform = carla.Transform(carla.Location(x=2, y=0.5, z=1.7), carla.Rotation(pitch=0.0, yaw=0.0))
        right_cam = world.spawn_actor(cam_bp, right_cam_transform,attach_to=ego_vehicle)
        left_cam.listen(lambda image: self.publish_right_image(image))
        right_cam.listen(lambda image: self.publish_left_image(image))
        # --------------
        # Add GNSS sensor to ego vehicle. 
        # --------------
        gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')
        gnss_location = carla.Location(0,0,0)
        gnss_rotation = carla.Rotation(0,0,0)
        gnss_transform = carla.Transform(gnss_location,gnss_rotation)
        #gnss_bp.set_attribute("sensor_tick",str(1))
        gnss_bp.set_attribute("noise_lat_bias",str(0.00001))
        gnss_bp.set_attribute("noise_lat_stddev",str(0.000005))
        gnss_bp.set_attribute("noise_lon_bias",str(0.00001))
        gnss_bp.set_attribute("noise_lon_stddev",str(0.000005))
        ego_gnss = world.spawn_actor(gnss_bp,gnss_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        ego_gnss.listen(lambda gnss: self.publish_GNSS(gnss))
        # --------------
        # Add IMU sensor to ego vehicle. 
        # --------------
        imu_bp = world.get_blueprint_library().find('sensor.other.imu')
        imu_location = carla.Location(0,0,0)
        imu_rotation = carla.Rotation(0,0,0)
        imu_transform = carla.Transform(imu_location,imu_rotation)
        ego_imu = world.spawn_actor(imu_bp,imu_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        ego_imu.listen(lambda imu: self.publish_IMU(imu))
        

        frame = 0
        while frame < 100000:
            self.publish_odometry(ego_vehicle)
            ego_vehicle.set_autopilot(True) 
            world.get_spectator().set_transform(carla.Transform(ego_vehicle.get_transform().transform(carla.Location(x=-7, z=4)),  ego_vehicle.get_transform().rotation))
            world.tick()           
            print(frame)
            frame += 1



    def publish_GNSS(self, gnss):
        gnss_msg = NavSatFix()
        gnss_msg.header = Header(stamp=self.get_clock().now().to_msg())
        gnss_msg.altitude = gnss.altitude
        gnss_msg.longitude = gnss.longitude
        gnss_msg.latitude = gnss.latitude
        self.GNSS_publisher.publish(gnss_msg)

    def publish_IMU(self, imu):
        imu_msg = Imu()
        imu_msg.header = Header(stamp=self.get_clock().now().to_msg())
        imu_msg.angular_velocity.x = imu.gyroscope.x
        imu_msg.angular_velocity.y = imu.gyroscope.y
        imu_msg.angular_velocity.z = imu.gyroscope.z
        imu_msg.linear_acceleration.x = imu.accelerometer.x
        imu_msg.linear_acceleration.y = imu.accelerometer.y
        imu_msg.linear_acceleration.z = imu.accelerometer.z
        imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=imu.compass, w=1.0)
        self.IMU_publisher.publish(imu_msg)

    def publish_odometry(self, ego_vehicle):
        odometry_msg = Odometry()
        odometry_msg.header = Header(stamp=self.get_clock().now().to_msg())

        # Set the pose information
        ego_transform = ego_vehicle.get_transform()
        odometry_msg.pose.pose.position = Point(x=ego_transform.location.x,
                                               y=ego_transform.location.y,
                                               z=ego_transform.location.z)
        odometry_msg.pose.pose.orientation =  Quaternion(x=ego_transform.rotation.yaw, y=ego_transform.rotation.pitch, z= ego_transform.rotation.roll, w=1.0) 
        
    
        # Set the twist information
        ego_velocity = ego_vehicle.get_velocity()
        odometry_msg.twist.twist.linear.x = ego_velocity.x
        odometry_msg.twist.twist.linear.y = ego_velocity.y
        odometry_msg.twist.twist.linear.z = ego_velocity.z
        odometry_msg.twist.twist.angular.x = ego_vehicle.get_angular_velocity().x
        odometry_msg.twist.twist.angular.y = ego_vehicle.get_angular_velocity().y
        odometry_msg.twist.twist.angular.z = ego_vehicle.get_angular_velocity().z

        # Publish the odometry message
        self.ODOMETRY_publisher.publish(odometry_msg)

    def publish_right_image(self, image):
        # Convert the image data to a NumPy array
        image_data = np.frombuffer(image.raw_data, dtype=np.uint8)
        # Reshape the array to match the dimensions of the image
        image_array = image_data.reshape((image.height, image.width, 4))
        # Convert the image to bgr
        rgb_image = image_array[:, :, :3]
        image_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.RIGHT_IMAGE_publisher.publish(image_msg)

    def publish_left_image(self, image):
        # Convert the image data to a NumPy array
        image_data = np.frombuffer(image.raw_data, dtype=np.uint8)
        # Reshape the array to match the dimensions of the image
        image_array = image_data.reshape((image.height, image.width, 4))
        # Convert the image to bgr
        rgb_image = image_array[:, :, :3]
        image_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.LEFT_IMAGE_publisher.publish(image_msg)

def main(args=None):
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=20,
        type=int,
        help='Number of vehicles (default: 20)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=20,
        type=int,
        help='Number of walkers (default: 20)')
    argparser.add_argument(
        '--seedw',
        metavar='S',
        default=0,
        type=int,
        help='Set the seed for pedestrians module')
    argparser.add_argument(
        '-pr','--percentagePedestriansRunning',
        metavar='PR',
        default=0,
        type=float,
        help='Set percentage Pedestrians Running 0-1')    
    argparser.add_argument(
        '-pc','--percentagePedestriansCrossing',
        metavar='PC',
        default=0,
        type=float,
        help='Set percentage Pedestrians Crossing 0-1')   
    args = argparser.parse_args()
    rclpy.init()
    Carla_ROS_Publisher = CarlaROSPublisher(args)
    rclpy.spin(Carla_ROS_Publisher)
    Carla_ROS_Publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
