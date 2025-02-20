from flask import Flask, render_template, Response, request, jsonify
from collections import deque
from ultralytics import YOLO  # Import YOLO
import threading
import time
import cv2

app = Flask(__name__)

# Initialize the webcam
camera = cv2.VideoCapture(0)

# Try importing ROS
USE_ROS = True
try:
    import rclpy
    from web_support import WebSupport
except ImportError:
    print("ROS not available. Running in non-ROS mode.")
    USE_ROS = False

if USE_ROS:
    from web_support import WebSupport
    # Initialize ROS in a separate thread
    rclpy.init()
    ros_node = WebSupport()
    def ros_spin():
        rclpy.spin(ros_node)
    threading.Thread(target=ros_spin, daemon=True).start()
else:
    from web_support_no_ros import WebSupport
    ros_node = WebSupport()

# Store the last entered/known coordinate for the UI
target_coordinate = {"lat": 0, "lon": 0}
computer_gps = {"lat": 0, "lon": 0}


@app.route('/')
def index():
    """Render the main page."""
    return render_template('index.html')

def generate_frames():
    """Capture frames from the webcam and process them with YOLO."""
    import os
    #model_path = os.path.abspath("./YOLOv11_custom/model-drone1.pt")
    #model_path = os.path.abspath("./YOLOv11_custom/yolov11_custom.pt")
    model_path = os.path.abspath("./YOLOv11_custom/test_l_size.pt")
    model = YOLO(model_path)  # Load YOLO model
    frame_count = 0
    frame_times = deque(maxlen=300)

    while True:
        # Read a frame from the webcam
        success, frame = camera.read()
        if not success:
            break
        else:
            # Add frame
            frame_count += 1  # Increment frame count
            current_time = time.time()
            frame_times.append(current_time)

            # Calculate FPS for the last 10 seconds
            while frame_times and frame_times[0] < current_time - 10:
                frame_times.popleft()
            fps = len(frame_times) / 10.0
            
            # Perform YOLO inference on the frame
            results = model.predict(frame, conf=0.5, show_labels=True, show_conf=True, classes=[0], verbose=False)

            # Extract the processed frame
            for result in results:
                frame = result.plot()  # Draw YOLO detections on the frame

                for box in result.boxes:
                    # Publish CV box
                    ros_node.publish_cv_box(box)

            # Overlay frame count and timestamp
            overlay_text = f"Frame: {frame_count} | FPS: {fps:.2f}"
            cv2.putText(frame, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Encode the frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            # Yield the processed frame
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """Route to stream video feed."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/control', methods=['POST'])
def control():
    """Handle control actions from the web app."""
    data = request.json
    steering = data.get('steering')
    speed = data.get('speed')
    xAxis = data.get('xAxis')
    yAxis = data.get('yAxis')
    print(f"___________________X:{xAxis}, Y:{yAxis}, DATA: {data}")
    ros_node.publish_control(steering, speed)  # Publish the action to the ROS topic
    ros_node.publish_servo_arm(xAxis, yAxis) # TODO replace with x_axis and y_axis and add to controls
    return jsonify({'status': 'success', 'action': data}), 200

@app.route('/set_computer_gps', methods=['POST'])
def set_computer_gps():
    """Receive computer GPS from map.js and store it"""
    global computer_gps
    try:
        data = request.get_json()
        computer_gps = {"lat": float(data["lat"]), "lon": float(data["lon"])}
        print(f"✅ Received Computer GPS: {computer_gps}")
        return jsonify({"status": "success", "computer_gps": computer_gps})
    except Exception as e:
        print(f"❌ Error receiving Computer GPS: {e}")
        return jsonify({"error": "Invalid GPS data"}), 400
    
@app.route('/get_node_data', methods=['GET'])
def get_node_data():
    global last_computer_gps  # Keep the last known GPS value
    
    """Ensure we get a dictionary, not a Flask Response."""
    raw_response = ros_node.get_node_data()

    # If raw_response is a Flask Response object, convert it to a dictionary
    if isinstance(raw_response, Response):
        try:
            raw_response = raw_response.get_json()  # Extract JSON
        except Exception as e:
            print("❌ Error parsing JSON from Response:", e)
            return jsonify({'error': 'Failed to parse node data'}), 500

    # Ensure the response is a dictionary
    if not isinstance(raw_response, dict):
        print(f"❌ Error: Unexpected data type: {type(raw_response)}")
        return jsonify({'error': 'Invalid node data format'}), 500

    # Convert car_gps from string to dictionary if necessary
    if isinstance(raw_response.get("car_gps"), str):
        try:
            lat, lon = map(float, raw_response["car_gps"].split(","))
            raw_response["car_gps"] = {"lat": lat, "lon": lon}
        except ValueError:
            print("❌ Error: Invalid GPS format in car_gps")
            raw_response["car_gps"] = {"lat": 0, "lon": 0}  # Default if parsing fails
    
    # ✅ Ensure `target_coordinate` is always valid
    if "target_coordinate" not in raw_response or raw_response["target_coordinate"] is None:
        print("⚠️ `target_coordinate` missing, setting to default (0,0)")
        raw_response["target_coordinate"] = {"lat": 0, "lon": 0}  # Default instead of undefined

    # ✅ Debugging: Print the exact response from ROS
    print(f"🔍 Raw Data from ROS: {raw_response}")

    # ✅ Include computer GPS from global variable
    raw_response["computer_gps"] = computer_gps
    
    # Ensure other GPS fields are dictionaries
    raw_response["target_coordinate"] = target_coordinate
    
    print("✅ Processed Node Data:", raw_response)  # Debugging
    return jsonify(raw_response)

@app.route('/set_target_coordinate', methods=['POST'])
def set_target_coordinate():
    global target_coordinate
    data = request.json
    coordinate = data.get('coordinate')

    try:
        lat, lon = map(float, coordinate.split(","))
        target_coordinate = {"lat": lat, "lon": lon}  # ✅ Store locally for UI
        print(f"✅ Target Coordinate Updated: {target_coordinate}")

        """Send the coordinate to ros node."""
        node_data = ros_node.update_target_coordinate(coordinate)
        if node_data is None:
            return jsonify({'status': 'failed'}), 500
        return jsonify({'status': 'success'}), 200

    except ValueError:
        print("❌ Invalid coordinate format received.")
        return jsonify({'status': 'failed', 'error': 'Invalid coordinate format'}), 400
    
@app.route('/set_control_mode', methods=['POST'])
def set_control_mode():
    data = request.json
    mode = data.get('mode')
    """Send the mode to ros node."""
    node_data = ros_node.set_control_mode(mode)
    if node_data is None:
        return jsonify({'status': 'failed'}), 500
    return jsonify({'status': 'success'}), 200

@app.route('/set_slider_position', methods=['POST'])
def set_slider_position():
    data = request.json
    x = data.get('x')
    y = data.get('y')

    if x is None or y is None:
        return jsonify({"status": "error", "message": "Invalid data"}), 400

    print(f"🎮 Servo Position Updated: X={x}, Y={y}")

    # TODO: Send x and y values to servos via ROS
    # ros_node.update_servos(x, y)

    return jsonify({"status": "success", "x": x, "y": y})
    
if __name__ == '__main__':    
    app.run(debug=False, host='0.0.0.0', port=5001)
    camera.release()
    if USE_ROS:
        ros_node.destroy_node()
        rclpy.shutdown()