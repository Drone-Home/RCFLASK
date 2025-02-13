# Links flask web and ROS nodes

# Collects data from multiple nodes and sends to website with ROS_TO_WEB topic
# Takes in data from website with WEB_TO_ROS topic
# Created by flask app.py
import rclpy
import threading
import math
from time import sleep
from math import degrees, radians
from flask import jsonify
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Quaternion, PoseStamped
from custom_messages.msg import CV 
from custom_messages.srv import SetCoordinate
from custom_messages.srv import SetMode 
from std_msgs.msg import String

class WebSupport(Node):
    def __init__(self):
        # Node data for web
        # Node data getter and setter
        self.node_data = {
            "computer_gps": "Waiting...",
            "car_yaw": "Wainting...",
            "drone_gps": "Waiting...",
            "car_gps": "Waiting...",
            "car_satellites": "Waiting...",
            "car_drive_status": "Waiting...",
            "battery_level": "Waiting..."
        }

        self.target_coordinate = NavSatFix(latitude=29.6449, longitude=-82.3481)
        self.control_mode = "manual" # TODO load to and from file on change or in ackermann drive
        
        super().__init__('web_support')

        self.client = self.create_client(SetCoordinate, 'drone_home1/set_target_coordinate')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Coordinate service not available, waiting again...')

        self.client2 = self.create_client(SetMode, 'drone_home1/set_control_mode')
        while not self.client2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Control mode service not available, waiting again...')

        # Subscribers for webdata
        self.gps_subscriber = self.create_subscription(NavSatFix, 'drone_home1/gps/fix', self.gps_callback, 10)
        self.pose_subscriber = self.create_subscription(PoseStamped, 'drone_home1/vehicle/pose', self.pose_callback, 10)
        self.drive_status_subscriber = self.create_subscription(String, 'drone_home1/vehicle/drive_status', self.drive_status_callback, 10)

        # Publisher for web steering
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drone_home1/vehicle/web_controller_drive', 10)
        
        # Publisher for CV
        self.cv_publisher_ = self.create_publisher(CV, 'drone_home1/vehicle/cv', 10)

        # Class variables updated by subscribers
        self.current_position = NavSatFix()
        self.drone_position = NavSatFix()
        self.current_quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
        self.current_drive_status = String(data="Waiting...")

        self.i = 0

        print("Web support started")

    def gps_callback(self, msg):
        # Update the current GPS location from NavSatFix message
        self.current_position = msg
        self.node_data.update({"car_gps":f"{self.current_position.latitude}, {self.current_position.longitude}"})
        self.node_data.update({"car_satellites":f"{self.current_position.status.service}"})
        #print(self.current_position)

    def pose_callback(self, msg):
        # Update the current quaternion from the PoseStamped message
        self.current_quaternion = msg.pose.orientation
        current_yaw = self.euler_from_quaternion(self.current_quaternion)[2]
        self.node_data.update({"car_yaw": f"{round(degrees(current_yaw), 2)} degrees"})

    def drive_status_callback(self, msg):
        # Update the current drive status from the message
        self.current_drive_status = msg.data
        self.node_data.update({"car_drive_status": f"{self.current_drive_status}"})

    def publish_cv_box(self, box):
        x1, y1, x2, y2 = box.xyxy[0]  # Bounding box corners
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        width = x2 - x1
        height = y2 - y1
        
        #print(f"Center: ({center_x:.2f}, {center_y:.2f}), Size: ({width:.2f}, {height:.2f})")
        msg = CV()
        msg.x_pos = float(center_x)
        msg.y_pos = float(center_y)
        msg.x_size = float(width)
        msg.y_size = float(height)
        msg.image_number = self.i
        self.cv_publisher_.publish(msg)

        self.i += 1

    def publish_control(self, steering, speed):
        # Map from web commands to drive commands and publish
        steering_angle = self.map_value(steering, -1, 1, -math.pi/4, math.pi/4)
        drive_speed = self.map_value(speed, -1, 1, -1, 1)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = drive_speed  
        self.publisher_.publish(drive_msg)

    def get_node_data(self):
        # Called by app.py, sends updated data
        return jsonify(self.node_data)
    
    def update_target_coordinate(self, coordiantes):
        latitude, longitude = self.parse_and_verify_lat_lon(coordiantes)
        if (latitude is not None and longitude is not None):
            print(f"Got coordinates {latitude, longitude}")
            # Update node coordinate
            self.target_coordinate.latitude = latitude
            self.target_coordinate.longitude = longitude

            # Send service update
            self.update_coordinate_service()

            return latitude, longitude
        return None

    def set_control_mode(self, control_mode):
        print(f"Got control mode {control_mode}")
        # Update node coordinate
        self.control_mode = control_mode

        # Send service update
        self.set_control_mode_service()

        return control_mode
    
    def update_coordinate_service(self):
        # Create the request message
        request = SetCoordinate.Request()
        print(f"{self.target_coordinate}")
        request.new_coordinate = self.target_coordinate

        # Send the request
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=5.0)

        # Check the response
        if self.future.result() is not None:
            self.get_logger().info(f'Service call succeeded: {self.future.result().success}')
            return self.future.result().success
        else:
            self.get_logger().error('Service call failed')
            return False
        
    def set_control_mode_service(self):
        # Create the request message
        request = SetMode.Request()
        print(f"{self.control_mode}")
        request.mode = self.control_mode

        # Send the request
        self.future = self.client2.call_async(request)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=5.0)

        # Check the response
        if self.future.result() is not None:
            self.get_logger().info(f'Service call succeeded: {self.future.result().success}')
            return self.future.result().success
        else:
            self.get_logger().error('Service call failed')
            return False

    def parse_and_verify_lat_lon(self, input_str):
        try:
            # Split the input string by comma
            lat_str, lon_str = input_str.strip().split(',')

            # Convert to float
            latitude = float(lat_str)
            longitude = float(lon_str)

            # Validate latitude and longitude ranges
            if not (-90 <= latitude <= 90):
                raise ValueError(f"Latitude {latitude} is out of range. Must be between -90 and 90.")
            if not (-180 <= longitude <= 180):
                raise ValueError(f"Longitude {longitude} is out of range. Must be between -180 and 180.")

            return latitude, longitude

        except ValueError as e:
            print(f"Invalid input: {input_str}.Error: {e}")
            return None, None
    
    def euler_from_quaternion(self, quaternion):
        w = quaternion.w
        z = quaternion.z
        x = quaternion.x
        y = quaternion.y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    @staticmethod
    def map_value(value, from_min, from_max, to_min, to_max):
        # Map value to new range
        mapped_value = to_min + (value - from_min) * (to_max - to_min) / (from_max - from_min)
        return float(mapped_value)
    
def main(args=None):
    rclpy.init()
    web_support = WebSupport() 
    def ros_spin():
        rclpy.spin(web_support) 
    threading.Thread(target=ros_spin, daemon=True).start()


    while True:
        sleep(5)

if __name__ == '__main__':
    main()