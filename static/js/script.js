document.addEventListener('DOMContentLoaded', function () {
    const leftButton = document.getElementById('left-btn');
    const rightButton = document.getElementById('right-btn');
    const lever = document.getElementById('lever');
    const leverContainer = document.querySelector('.lever-container');
    const statusDisplay = document.getElementById('lever-status');
    const outputLog = document.getElementById('output-log'); // Output window log
    const autodriveButton = document.getElementById('autodrive-btn');
    const updateCoordinateButton = document.getElementById('update-coordinate-btn');
    const manualControlRadio = document.querySelector('input[value="manual"]');
    const automaticControlRadio = document.querySelector('input[value="automatic"]'); 

    let actionIndex = 1; // Tracks number of actions
    let steering = 0; // -1 (left), 0 (neutral), 1 (right)
    let speed = 0; // -1 (full reverse) to 1 (full forward)
    let xAxis = 1; // charging cable servo -1 (left), 0 (neutral), 1 (right)
    let yAxis = 1; // charging cable -1 (down), 0 (neutral), 1 (up)
    let isAutoDrive = false;

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
    
            logCommand(actionData);
    
            // Store last values to prevent redundant updates
            lastSteering = steering;
            lastSpeed = speed;
            lastXAxis = xAxis;
            lastYAxis = yAxis;
        }
    }

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

    function logCommand(commandData) {
        const logEntry = document.createElement('div');
        logEntry.innerHTML = `<span>Action #${actionIndex}:</span> 
                              <span>Steering: ${commandData.steering}, Speed: ${commandData.speed.toFixed(2)}</span>`;
        logEntry.classList.add('log-entry');

        outputLog.appendChild(logEntry);
        outputLog.scrollTop = outputLog.scrollHeight;
        actionIndex++; // Increment action count
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

    manualControlRadio.addEventListener('change', function () {
        setControlMode('manual');
    });
    automaticControlRadio.addEventListener('change', function () {
        setControlMode('automatic');
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

});

// Store latest received data
let receivedData = {
    computer_gps: null,
    car_gps: null,
    drone_gps: null,
};

// Function to calculate distance between two GPS coordinates (Haversine Formula)
function calculateDistance(lat1, lon1, lat2, lon2) {
    const R = 6371e3; // Earth radius in meters
    const toRadians = (deg) => deg * (Math.PI / 180);

    const φ1 = toRadians(lat1);
    const φ2 = toRadians(lat2);
    const Δφ = toRadians(lat2 - lat1);
    const Δλ = toRadians(lon2 - lon1);

    const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
              Math.cos(φ1) * Math.cos(φ2) *
              Math.sin(Δλ / 2) * Math.sin(Δλ / 2);

    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return (R * c).toFixed(2); // Distance in meters
}

// Function to update UI values dynamically
function updateReceivedData(newData) {
    
    updateNodeData();
    if (newData.computer_gps) {
        receivedData.computer_gps = newData.computer_gps;
        document.getElementById("computer-gps").textContent = 
        `${newData.computer_gps.lat}, ${newData.computer_gps.lon}`;
       }
       
    /*
    if (newData.drone_gps) {
        receivedData.drone_gps = newData.drone_gps;
        document.getElementById("drone-gps").textContent = 
        `${newData.drone_gps.lat}, ${newData.drone_gps.lon}`;
    }

    if (newData.car_gps) {
        receivedData.car_gps = newData.car_gps;
        document.getElementById("car-gps").textContent = 
        `${newData.car_gps.lat}, ${newData.car_gps.lon}`;
    }
    */

    // Calculate distances without waiting for battery data
    if (receivedData.computer_gps && receivedData.car_gps) {
        document.getElementById("distance-computer-car").textContent = 
            calculateDistance(receivedData.computer_gps.lat, receivedData.computer_gps.lon, 
                              receivedData.car_gps.lat, receivedData.car_gps.lon) + " m";
    }

    if (receivedData.car_gps && receivedData.drone_gps) {
        document.getElementById("distance-car-drone").textContent = 
            calculateDistance(receivedData.car_gps.lat, receivedData.car_gps.lon, 
                              receivedData.drone_gps.lat, receivedData.drone_gps.lon) + " m";
    }

    if (receivedData.drone_gps && receivedData.computer_gps) {
        document.getElementById("distance-drone-computer").textContent = 
            calculateDistance(receivedData.drone_gps.lat, receivedData.drone_gps.lon, 
                              receivedData.computer_gps.lat, receivedData.computer_gps.lon) + " m";
    }
}

function updateNodeData() {
    fetch('/get_node_data')
        .then(response => response.json())
        .then(data => {
            //document.getElementById('computer-gps').textContent = data.computer_gps;
            document.getElementById('drone-gps').textContent = data.drone_gps;
            document.getElementById('car-gps').textContent = data.car_gps.lat + ", " + data.car_gps.lon;
            document.getElementById('car-yaw').textContent = data.car_yaw;
            document.getElementById('car-satellites').textContent = data.car_satellites;
            document.getElementById('car-drive-status').textContent = data.car_drive_status;
            //document.getElementById('battery-level').textContent = data.battery_level;
        })
        .catch(error => console.error('Error fetching node data:', error));
}

// Refresh node data 
setInterval(updateReceivedData, 300);

// Load on page
document.addEventListener("DOMContentLoaded", updateReceivedData);

// Listen for "gpsLocationUpdate" from `map.js`
window.addEventListener("gpsLocationUpdate", (event) => {
    updateReceivedData(event.detail);
});