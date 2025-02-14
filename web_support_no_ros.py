from flask import jsonify
from time import sleep
# Links flask web and ROS nodes

# Collects data from multiple nodes and sends to website with ROS_TO_WEB topic
# Takes in data from website with WEB_TO_ROS topic
# Created by flask app.py

class WebSupport():
    def __init__(self):
        # Node data for web
        # Node data getter and setter
        self.node_data = {
            "computer_gps": "Waiting...",
            "car_yaw": "180.0",
            "drone_gps": "Waiting...",
            "car_gps": "29.632671, -82.36249",
            "car_satellites": "5",
            "car_drive_status": "Test status no ros",
            "battery_level": "Waiting..."
        }
        

        print("Web support started")

    def publish_cv_box(self, box):
        pass
    
    def publish_control(self, steering, speed):
        pass

    def publish_servo_arm(self, x_axis, y_axis):
        pass

    def get_node_data(self):
        # Called by app.py, sends updated data
        return jsonify(self.node_data)
    
    def update_target_coordinate(self, coordiantes):
        latitude, longitude = self.parse_and_verify_lat_lon(coordiantes)
        if (latitude is not None and longitude is not None):
            print(f"Got coordinates {latitude, longitude}")
            return latitude, longitude
        return None

    def set_control_mode(self, control_mode):
        print(f"Got control mode {control_mode}")
        return control_mode

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
    

    @staticmethod
    def map_value(value, from_min, from_max, to_min, to_max):
        # Map value to new range
        mapped_value = to_min + (value - from_min) * (to_max - to_min) / (from_max - from_min)
        return float(mapped_value)
    
def main(args=None):
    web_support = WebSupport()
    while True:
        sleep(5)

if __name__ == '__main__':
    main()