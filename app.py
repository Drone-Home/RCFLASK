from flask import Flask, render_template, Response, request, jsonify
from collections import deque
from ultralytics import YOLO  # Import YOLO
from usb_monitor import SerialMonitor
import threading
import queue
import time
import cv2

app = Flask(__name__)

# Initialize the webcam
main_camera = cv2.VideoCapture(0)
#probe_camera = cv2.VideoCapture(2)
probe_enabled = False # API changes which camera frames are taken from
last_probe_enabled = False # Track changes
CAMERA_ENABLED = True # Constant that disables camera entirely
# Queue for processed frames
frame_queue = queue.Queue(maxsize=5) 


# Try importing ROS
USE_ROS = True
try:
    import rclpy
    from web_support import WebSupport
except ImportError:
    print("ROS not available. Running in non-ROS mode.")
    USE_ROS = True

if USE_ROS:
    from web_support import WebSupport
    # Initialize ROS in a separate thread
    rclpy.init()
    ros_node = WebSupport()
    def ros_spin():
        rclpy.spin(ros_node)
    threading.Thread(target=ros_spin, daemon=True).start()
    # USB Charger monitor
    serial_monitor = SerialMonitor('/dev/ttyACM0', 115200)
    serial_monitor.open_serial_connection()
else:
    from web_support_no_ros import WebSupport
    ros_node = WebSupport()

@app.route('/')
def index():
    """Render the main page."""
    return render_template('index.html')

def capture_and_process_frames():
    """Capture frames from the webcam and process them with YOLO."""
    import os
    model_path = os.path.abspath("./YOLOv11_custom/model-droneV3-l.pt")
    #model_path = os.path.abspath("./YOLOv11_custom/test_l_size.pt")
    model = YOLO(model_path)  # Load YOLO model
    frame_count = 0
    frame_times = deque(maxlen=300)

    while True:
        global last_probe_enabled
        global main_camera
        if probe_enabled != last_probe_enabled:
            if probe_enabled:
                main_camera.release()
                main_camera = cv2.VideoCapture(2)
            else:
                main_camera.release()
                main_camera = cv2.VideoCapture(0) # 0
            last_probe_enabled = probe_enabled


        success, frame = main_camera.read()   
        # Read a frame from the selected webcam
        #if probe_enabled:
        #    success, frame = probe_camera.read()
        #else:
        #    success, frame = main_camera.read()

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
            results = model.predict(frame, conf=0.6, show_labels=True, show_conf=True, classes=[0], verbose=False)
            
            # Extract the processed frame
            for result in results:
                frame = result.plot()  # Draw YOLO detections on the frame

                for box in result.boxes:
                    # Publish CV box
                    ros_node.publish_cv_box(box)

            # Overlay frame count and timestamp
            overlay_text = f"FPS: {fps:.2f}" # Frame: {frame_count} | 
            cv2.putText(frame, overlay_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Overlay charger connection status
            encoding_quality = 8 # Adjust quality (1-100) 9
            if probe_enabled:
                encoding_quality = 9 # 15
                # conditionally better quality and voltage overlay with probe cam active
                if USE_ROS:
                    serial_monitor.monitor_non_blocking()
                    overlay_voltage = serial_monitor.get_latest_data()
                else:
                    overlay_voltage = "No ROS"
                # Draw black text (outline) in all directions
                cv2.putText(frame, f"{overlay_voltage}", (10-1, frame.shape[0] - 10-1), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(frame, f"{overlay_voltage}", (10+1, frame.shape[0] - 10-1), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(frame, f"{overlay_voltage}", (10-1, frame.shape[0] - 10+1), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(frame, f"{overlay_voltage}", (10+1, frame.shape[0] - 10+1), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 4, cv2.LINE_AA)

                # Draw the green text on top (centered)
                cv2.putText(frame, f"{overlay_voltage}", (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 4, cv2.LINE_AA)



            # Encode the frame as JPEG
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, encoding_quality]  # Adjust quality (1-100) 9
            _, buffer = cv2.imencode('.jpg', frame, encode_params)
            frame = buffer.tobytes()

            # Put frame in queue
            if not frame_queue.full():
                frame_queue.put(frame)
            else:
                frame_queue.get()  # Remove old frame
                frame_queue.put(frame)

# Start frame processing in a separate thread. Solves issue of slow connection making FPS drop. Camera runs at full speed and frames are served from queue
if CAMERA_ENABLED:
    threading.Thread(target=capture_and_process_frames, daemon=True).start()

# Frame for before camera is loaded
def create_placeholder_frame():
    """Create a blank frame"""
    width = 640
    height = 480
    black_frame = cv2.rectangle(
        img=cv2.UMat(height, width, cv2.CV_8UC3),  # Create an empty image
        pt1=(0, 0),
        pt2=(width, height),
        color=(0, 0, 0),  # Black color
        thickness=-1  # Fill the rectangle
    ).get()

    cv2.putText(black_frame, "CAMERA LOADING ...", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 
                1, (255, 255, 255), 2, cv2.LINE_AA)  # White text
    _, buffer = cv2.imencode('.jpg', black_frame)  # Encode as JPEG
    return buffer.tobytes()

def generate_frames():
    """Continuously send the latest available frame if there is one."""
    first_frame_loaded = False  # Flag to indicate if the first frame has been loaded
    while True:
        if not frame_queue.empty():  # Check if there are frames available
            frame = frame_queue.get()  
            first_frame_loaded = True
        else:
            if not first_frame_loaded:
                frame = create_placeholder_frame()  # Create a placeholder frame if no frames are available
            time.sleep(0.01)     
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
    ros_node.publish_control(steering, speed)  # Publish the action to the ROS topic
    ros_node.publish_servo_arm(xAxis, yAxis)
    return jsonify({'status': 'success', 'action': data}), 200

@app.route('/get_node_data', methods=['GET'])
def get_node_data():
    """get node data from web support and send json"""
    raw_response = ros_node.get_node_data()

    try:
        response = raw_response.get_json()
    except Exception as e:
        print("Error converting get_node_data to JSON:", e)
        return jsonify({'error': 'Invalid node data format'}), 500

    #print("Processed Node Data:", response)  # Debugging
    return response

@app.route('/set_target_coordinate', methods=['POST'])
def set_target_coordinate():
    data = request.json
    coordinate = data.get('coordinate')
    """Send the coordinate to ros node."""
    node_data = ros_node.update_target_coordinate(coordinate)
    if node_data is None:
        return jsonify({'status': 'failed'}), 500
    return jsonify({'status': 'success'}), 200

@app.route('/set_control_mode', methods=['POST'])
def set_control_mode():
    data = request.json
    mode = data.get('mode')
    """Send the mode to ros node."""
    node_data = ros_node.set_control_mode(mode)
    if node_data is None:
        return jsonify({'status': 'failed'}), 500
    return jsonify({'status': 'success'}), 200

@app.route('/set_camera', methods=['get'])
def set_camera():
    global probe_enabled
    probe_enabled = not probe_enabled
    """Handle switching cameras."""
    
    return jsonify({'status': 'success'}), 200
    
if __name__ == '__main__':    
    app.run(debug=False, host='0.0.0.0', port=5001)
    main_camera.release()
    if USE_ROS:
        ros_node.destroy_node()
        rclpy.shutdown()