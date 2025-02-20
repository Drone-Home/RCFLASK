function haversineDistance(lat1, lon1, lat2, lon2) {
    const R = 6371e3; // Earth radius in meters
    const toRadians = (deg) => (deg * Math.PI) / 180;

    const φ1 = toRadians(lat1);
    const φ2 = toRadians(lat2);
    const Δφ = toRadians(lat2 - lat1);
    const Δλ = toRadians(lon2 - lon1);

    const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
              Math.cos(φ1) * Math.cos(φ2) *
              Math.sin(Δλ / 2) * Math.sin(Δλ / 2);

    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c; // Distance in meters
}

document.addEventListener('DOMContentLoaded', function () {
    console.log("✅ map.js loaded successfully");

    // Initialize the map
    const map = L.map('map').setView([0, 0], 15); // Default center

    // Add a satellite tile layer
    L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
        attribution: '© Esri, Maxar, Earthstar Geographics, and the GIS User Community'
    }).addTo(map);

    let carMarker, droneMarker, computerMarker, targetMarker;
    let lastTargetCoordinate = null;

    function addOrUpdateMarker(marker, lat, lon, popupText) {
        if (!lat || !lon || lat === 0 || lon === 0) return marker;

        if (marker) {
            marker.setLatLng([lat, lon]); // Move existing marker
        } else {
            marker = L.marker([lat, lon]).addTo(map).bindPopup(popupText);
        }
        return marker;
    }

    // Fetch car GPS data from Flask
    function fetchAndUpdateGPS() {
        console.log("🔄 Fetching GPS data...");
        fetch('/get_node_data')
            .then(response => response.json())
            .then(data => {
                console.log("📍 Received GPS data:", data);

                if (data.error) {
                    console.error("❌ Error from server:", data.error);
                    return;
                }

                let bounds = []; // Store all valid marker positions
                let shouldResize = false; // Only resize when target coordinate changes

                // ✅ Debugging: Log all GPS values
                console.log(`📡 Car GPS: ${JSON.stringify(data.car_gps)}`);
                console.log(`📡 Drone GPS (Target): ${JSON.stringify(data.target_coordinate)}`);
                console.log(`📡 Computer GPS: ${JSON.stringify(data.computer_gps)}`);
                
                let carLat = data.car_gps?.lat || 0, carLon = data.car_gps?.lon || 0;
                let droneLat = data.target_coordinate?.lat || 0, droneLon = data.target_coordinate?.lon || 0;
                let computerLat = data.computer_gps?.lat || 0, computerLon = data.computer_gps?.lon || 0;

                // ✅ Check if distances can be calculated
                if (carLat && carLon && droneLat && droneLon && computerLat && computerLon) {
                    let distanceComputerCar = haversineDistance(computerLat, computerLon, carLat, carLon);
                    let distanceCarDrone = haversineDistance(carLat, carLon, droneLat, droneLon);
                    let distanceDroneComputer = haversineDistance(droneLat, droneLon, computerLat, computerLon);

                    console.log(`📏 Distances Updated:
                        Computer → Car: ${distanceComputerCar.toFixed(2)}m
                        Car → Drone: ${distanceCarDrone.toFixed(2)}m
                        Drone → Computer: ${distanceDroneComputer.toFixed(2)}m
                    `);

                    // ✅ Send distances to script.js
                    window.dispatchEvent(new CustomEvent("gpsLocationUpdate", {
                        detail: {
                            distanceComputerCar,
                            distanceCarDrone,
                            distanceDroneComputer
                        }
                    }));
                } else {
                    console.warn("⚠️ Missing GPS data, skipping distance calculation.");
                }

                // Update Car GPS
                if (data.car_gps && data.car_gps.lat !== 0) {
                    carMarker = addOrUpdateMarker(carMarker, data.car_gps.lat, data.car_gps.lon, "R/C Car GPS");
                    carLat = data.car_gps.lat;
                    carLon = data.car_gps.lon;
                    bounds.push([carLat, carLon]);
                }

                // ✅ Update Target Coordinate Marker
                if (data.target_coordinate && data.target_coordinate.lat !== 0) {
                    targetMarker = addOrUpdateMarker(targetMarker, data.target_coordinate.lat, data.target_coordinate.lon, "Target Coordinate");
                    console.log("🎯 Updating Target Coordinate on Map:", data.target_coordinate);

                    // 🔹 Fix: Ensure target coordinate is properly tracked
                    if (!lastTargetCoordinate || 
                        lastTargetCoordinate.lat !== data.target_coordinate.lat || 
                        lastTargetCoordinate.lon !== data.target_coordinate.lon) {
                        
                        console.log("📌 Target Coordinate Changed: Resizing Map");
                        shouldResize = true; // ✅ Now shouldResize is set to true
                        lastTargetCoordinate = { ...data.target_coordinate }; // ✅ Store the new target coordinate
                    }
                    droneLat = data.target_coordinate.lat;
                    droneLon = data.target_coordinate.lon;

                    bounds.push([data.target_coordinate.lat, data.target_coordinate.lon]); // ✅ Always track bounds
                }

                // ✅ Update Computer GPS
                if (data.computer_gps && data.computer_gps !== "Waiting..." && data.computer_gps.lat !== 0) {
                    computerMarker = addOrUpdateMarker(computerMarker, data.computer_gps.lat, data.computer_gps.lon, "Computer GPS");
                    computerLat = data.computer_gps.lat;
                    computerLon = data.computer_gps.lon;
                    bounds.push([computerLat, computerLon]);
                }

                // ✅ Calculate Distances (Only if we have all 3 points)
                if (computerLat && computerLon && droneLat && droneLon) {
                    let distanceComputerCar = haversineDistance(computerLat, computerLon, data.car_gps.lat, data.car_gps.lon);
                    let distanceCarDrone = haversineDistance(data.car_gps.lat, data.car_gps.lon, droneLat, droneLon);
                    let distanceDroneComputer = haversineDistance(droneLat, droneLon, computerLat, computerLon);

                    console.log(`📏 Distances Updated:
                        Computer → Car: ${distanceComputerCar.toFixed(2)}m
                        Car → Drone: ${distanceCarDrone.toFixed(2)}m
                        Drone → Computer: ${distanceDroneComputer.toFixed(2)}m
                    `);

                    // ✅ Send data to script.js
                    window.dispatchEvent(new CustomEvent("gpsLocationUpdate", {
                        detail: {
                            distanceComputerCar,
                            distanceCarDrone,
                            distanceDroneComputer
                        }
                    }));
                }

               // ✅ Resize map ONLY when a new target coordinate is set
               if (shouldResize && bounds.length > 0) {
                console.log("📌 Adjusting map view to fit target and related markers.");
                map.fitBounds(bounds, { padding: [50, 50] });

                // ✅ Fix: Force map refresh to prevent black screen issue
                setTimeout(() => {
                    map.invalidateSize();
                    console.log("🔄 Map refreshed after resizing.");
                }, 500);
            }
            
            // Dispatch event for other scripts
            window.dispatchEvent(new CustomEvent("gpsLocationUpdate", { detail: data }));
        })
        .catch(error => console.error("❌ Error fetching GPS data:", error));
    }

    // Fetch car GPS data every second
    if (typeof gpsUpdateInterval === "undefined") {
        console.log("✅ Starting GPS updates...");
        gpsUpdateInterval = setInterval(fetchAndUpdateGPS, 1000);
    }
    

    // Get the user's current location (Computer GPS)
    if (navigator.geolocation) {
        navigator.geolocation.getCurrentPosition(
            function (position) {
                const lat = position.coords.latitude;
                const lon = position.coords.longitude;

                // Center the map on the user's location
                map.setView([lat, lon], 15);

                // Set computer GPS dynamically
                const computerGPS = { lat, lon };
                computerMarker = addOrUpdateMarker(computerMarker, computerGPS.lat, computerGPS.lon, "Computer GPS");

                // Dispatch event with computer GPS
                window.dispatchEvent(new CustomEvent("gpsLocationUpdate", {
                    detail: { computer_gps: computerGPS }
                }));
            },
            function (error) {
                console.error("❌ Geolocation error: ", error);
                alert("Unable to fetch your location.");
            }
        );
    } else {
        alert("❌ Geolocation is not supported by your browser.");
    }
});
