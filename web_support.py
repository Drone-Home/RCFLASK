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
            "battery_level": "Waiting..."
        }
        
        super().__init__('web_support')
        # For webdata
        self.gps_subscriber = self.create_subscription(NavSatFix, 'drone_home1/gps/fix', self.gps_callback, 10)
        self.pose_subscriber = self.create_subscription(PoseStamped, 'drone_home1/vehicle/pose', self.pose_callback, 10)

        # Publisher for web steering
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drone_home1/vehicle/web_controller_drive', 10)
        
        # Publisher for CV
        self.cv_publisher_ = self.create_publisher(CV, 'drone_home1/vehicle/cv', 10)

        self.current_position = NavSatFix()
        self.drone_position = NavSatFix()
        self.current_quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)
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


        
    def publish_cv_box(self, box):
        x1, y1, x2, y2 = box.xyxy[0]  # Bounding box corners
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        width = x2 - x1
        height = y2 - y1

        
        # TODO Publish            
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

    sleep(30)

if __name__ == '__main__':
    main()