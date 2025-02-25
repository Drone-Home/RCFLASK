const controlModeSwitch = document.getElementById('control-mode-switch'); // Ensure this is defined

document.addEventListener('DOMContentLoaded', function () {
    const leftButton = document.getElementById('left-btn');
    const rightButton = document.getElementById('right-btn');
    const lever = document.getElementById('lever');
    const leverContainer = document.querySelector('.lever-container');
    const statusDisplay = document.getElementById('lever-status');
    const outputLog = document.getElementById('output-log'); // Output window log
    const autodriveButton = document.getElementById('autodrive-btn');
    const updateCoordinateButton = document.getElementById('update-coordinate-btn');
    

    let actionIndex = 1; // Tracks number of actions
    let steering = 0; // -1 (left), 0 (neutral), 1 (right)
    let speed = 0; // -1 (full reverse) to 1 (full forward)
    let xAxis = 0; // charging cable servo -1 (left), 0 (neutral), 1 (right)
    let yAxis = 0; // charging cable -1 (down), 0 (neutral), 1 (up)

    // Set initial neutral position for the lever
    const containerHeight = leverContainer.offsetHeight;
    const leverHeight = lever.offsetHeight;
    const neutralPosition = (containerHeight - leverHeight) / 2;
    lever.style.bottom = `${neutralPosition}px`;
    statusDisplay.textContent = "Neutral";

    function updateLeverStatus(currentPosition, maxHeight) {
        let positionPercentage = currentPosition / maxHeight;
    
        // Convert position to discrete steps of 0.2 (-1 to 1)
        let stepSize = 0.2;
        let scaledValue = Math.round((positionPercentage * 2 - 1) / stepSize) * stepSize;
    
        // Ensure speed stays within bounds (-1 to 1)
        speed = Math.max(-1, Math.min(1, scaledValue));
    
        if (speed > 0) {
            statusDisplay.textContent = "Forwards";
        } else if (speed < 0) {
            statusDisplay.textContent = "Backwards";
        } else {
            statusDisplay.textContent = "Neutral";
        }
    
        sendActionToServer(true); // Force update when speed changes
    }
    
    function adjustLeverPosition(direction) {
        const containerRect = leverContainer.getBoundingClientRect();
        const leverHeight = lever.offsetHeight;
        let currentPosition = parseInt(lever.style.bottom) || 0;
    
        // Move lever in increments of 0.2
        let stepSize = (containerRect.height - leverHeight) / 10; // 10 steps (-1 to 1)
    
        if (direction === 'up') {
            currentPosition = Math.min(containerRect.height - leverHeight, currentPosition + stepSize);
        } else if (direction === 'down') {
            currentPosition = Math.max(0, currentPosition - stepSize);
        }
    
        lever.style.bottom = `${currentPosition}px`;
        updateLeverStatus(currentPosition, containerRect.height - leverHeight);
    }

    let lastSteering = 0;
    let lastSpeed = 0;
    let lastXAxis = 0;
    let lastYAxis = 0;
    
    function sendActionToServer(forceUpdate = false) {
        const actionData = {
            steering: steering,
            speed: speed,
            xAxis: xAxis,
            yAxis: yAxis,
        };
    
        // Only send an update if something changed OR if forced (lever movement)
        if (forceUpdate || steering !== lastSteering || speed !== lastSpeed || xAxis !== lastXAxis || yAxis !== lastYAxis) {
            fetch('/control', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(actionData),
            })
            .then(response => response.json())
            .then(data => {
                console.log('Server response:', data);
            })
            .catch(error => {
                console.error('Error sending action to server:', error);
            });

            // set slider to manual
            controlModeSwitch.checked = true;

            //logCommand(actionData);
    
            // Store last values to prevent redundant updates
            lastSteering = steering;
            lastSpeed = speed;
            lastXAxis = xAxis;
            lastYAxis = yAxis;
        }
    }
    // Limit frequency of sending to avoid lag when called too frequently
    function throttle(func, wait) {
        let lastTime = 0;
        return function(...args) {
            const now = new Date().getTime();
            if (now - lastTime >= wait) {
                lastTime = now;
                func.apply(this, args);
            }
        };
    }
    const throttledSendActionToServer = throttle(sendActionToServer, 100);

    function adjustLeverPosition(direction) {
        const containerRect = leverContainer.getBoundingClientRect();
        const leverHeight = lever.offsetHeight;
        let currentPosition = parseInt(lever.style.bottom) || 0;

        if (direction === 'up') {
            currentPosition = Math.min(containerRect.height - leverHeight, currentPosition + 20);
        } else if (direction === 'down') {
            currentPosition = Math.max(0, currentPosition - 20);
        }

        lever.style.bottom = `${currentPosition}px`;
        updateLeverStatus(currentPosition, containerRect.height - leverHeight);
    }

    document.addEventListener('keydown', function (event) {
        // Prevent arrow keys from scrolling the page
        if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key)) {
            event.preventDefault();
        }

        if (!isDragging) {
            switch (event.key) {
                case 'ArrowLeft':
                    leftButton.classList.add('clicked');
                    steering = -1;
                    sendActionToServer();
                    break;
                case 'ArrowRight':
                    rightButton.classList.add('clicked');
                    steering = 1;
                    sendActionToServer();
                    break;
                case 'ArrowUp':
                    adjustLeverPosition('up');
                    break;
                case 'ArrowDown':
                    adjustLeverPosition('down');
                    break;
            }
        }
    });

    document.addEventListener('keyup', function (event) {
        // Prevent arrow keys from scrolling the page
        if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.key)) {
            event.preventDefault();
        }
        switch (event.key) {
            case 'ArrowLeft':
                leftButton.classList.remove('clicked');
                steering = 0;
                sendActionToServer();
                break;
            case 'ArrowRight':
                rightButton.classList.remove('clicked');
                steering = 0;
                sendActionToServer();
                break;
        }
    });

    // Lever dragging logic
    let isDragging = false;

    lever.addEventListener('mousedown', function () {
        isDragging = true;
    });

    document.addEventListener('mousemove', function (event) {
        if (isDragging) {
            const containerRect = leverContainer.getBoundingClientRect();
            const leverHeight = lever.offsetHeight;

            let newY = event.clientY - containerRect.top - leverHeight / 2;
            newY = Math.max(0, Math.min(newY, containerRect.height - leverHeight));

            lever.style.bottom = `${containerRect.height - newY - leverHeight}px`;
            updateLeverStatus(containerRect.height - newY - leverHeight, containerRect.height - leverHeight);
        }
    });

    document.addEventListener('mouseup', function () {
        isDragging = false;
    });

    leftButton.addEventListener('mousedown', function () {
        steering = -1;
        sendActionToServer();
    });
    
    rightButton.addEventListener('mousedown', function () {
        steering = 1;
        sendActionToServer();
    });
    
    document.addEventListener('mouseup', function () {
        steering = 0;
        sendActionToServer();
    });

    updateCoordinateButton.addEventListener('click', function () {
        // Get the raw input value from the coordinate text box
        const coordinateInput = document.getElementById('coordinate').value;

        // Send the raw input string to the server
        fetch('/set_target_coordinate', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ coordinate: coordinateInput }), // Send the raw string
        })
        .then(response => response.json())
        .then(data => {
            console.log('Server response:', data);
        })
        .catch(error => {
            console.error('Error sending coordinate to server:', error);
        });
    });

    controlModeSwitch.addEventListener('change', function () {
        if (this.checked) {
            setControlMode('manual');
        } else {
            setControlMode('automatic');
        }
    });

    // Function to send control mode to the server
    function setControlMode(mode) {
        fetch('/set_control_mode', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ mode: mode }),
        })
        .then(response => response.json())
        .then(data => {
            console.log('Control mode set:', data);
        })
        .catch(error => {
            console.error('Error setting control mode:', error);
        });
    }

    var slider = {
        get_position: function() {
            var marker_pos = $('#marker').position();
            var left_pos = marker_pos.left + slider.marker_size / 2;
            var top_pos = marker_pos.top + slider.marker_size / 2;
    
            // Convert to -1 to 1 range with 0.05 increments
            let raw_x = (left_pos * slider.xmax / slider.width) * 2 - 1;
            let raw_y = ((slider.height - top_pos) * slider.ymax / slider.height) * 2 - 1;
    
            slider.position = {
                x: Math.round(raw_x * 40) / 40,  // Round to 0.0025 increments
                y: Math.round(raw_y * 40) / 40
            };
    
            // Send data to Flask
            xAxis = slider.position.x;
            yAxis = slider.position.y;
            throttledSendActionToServer();
        },
    
        draw: function(x_size, y_size, xmax, ymax, marker_size) {
            slider.marker_size = marker_size;
            slider.width = x_size;
            slider.height = y_size;
            slider.xmax = xmax;
            slider.ymax = ymax;
    
            $("#markerbounds").css({
                "width": (x_size + marker_size) + 'px',
                "height": (y_size + marker_size) + 'px'
            });
    
            $("#box").css({
                "width": x_size + 'px',
                "height": y_size + 'px',
                "top": marker_size / 2,
                "left": marker_size / 2
            });
    
            $("#marker").css({
                "width": marker_size + 'px',
                "height": marker_size + 'px'
            });
    
            slider.get_position();
        }
    };
    
    // Enable dragging
    $("#marker").draggable({
        containment: "#markerbounds",
        drag: function() {
            slider.get_position();
        }
    });
    
    // Initialize slider (Size: 150x150, Range: -1 to 1, Marker Size: 20)
    slider.draw(150, 150, 1, 1, 20);


});

// Store latest received data
let receivedData = {
    computer_gps: null,
    car_gps: null,
    drone_gps: null,
};

let checkboxUpdated = false;

function updateNodeData() {
    fetch('/get_node_data')
        .then(response => response.json())
        .then(data => {
            document.getElementById('drone-gps').textContent = data.drone_gps.lat + ", " + data.drone_gps.lon;
            document.getElementById('car-gps').textContent = data.car_gps.lat + ", " + data.car_gps.lon;
            document.getElementById('car-yaw').textContent = data.car_yaw;
            document.getElementById('car-satellites').textContent = data.car_satellites;
            document.getElementById('car-drive-status').textContent = data.car_drive_status;

            // Update checkbox once to match loaded state
            if (!checkboxUpdated){
                console.log("Change modes from memory");
                if (data.car_mode === "manual") {
                    controlModeSwitch.checked = true;
                    checkboxUpdated = true;
                }
                else if (data.car_mode === "automatic") {
                    controlModeSwitch.checked = false;
                    checkboxUpdated = true;
                }   
            }  
        })
        .catch(error => console.error('Error fetching node data:', error));
}


// Refresh node data 
setInterval(updateNodeData, 600);

// Load on page
document.addEventListener("DOMContentLoaded", updateNodeData);

// Function to update distance values from map
function updateDistanceData(newData) {
    if (newData.distanceComputerCar && newData.distanceCarDrone && newData.distanceDroneComputer) {
        document.getElementById("distance-computer-car").textContent = 
            `${newData.distanceComputerCar.toFixed(2)} meters`;
        document.getElementById("distance-car-drone").textContent = 
            `${newData.distanceCarDrone.toFixed(2)} meters`;
        document.getElementById("distance-drone-computer").textContent = 
            `${newData.distanceDroneComputer.toFixed(2)} meters`;
    }
}

// Listen for distance updates from `map.js`
window.addEventListener("gpsLocationUpdate", (event) => {
    console.log("Got distance update");
    updateDistanceData(event.detail);
});

