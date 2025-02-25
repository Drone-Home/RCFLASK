document.addEventListener('DOMContentLoaded', function () {
    console.log("✅ map.js loaded successfully");

    // Initialize the map
    const map = L.map('map').setView([0, 0], 15); // Default center

    // Add a satellite tile layer
    L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
        attribution: '© Esri, Maxar, Earthstar Geographics, and the GIS User Community'
    }).addTo(map);

    let carMarker, droneMarker, computerMarker;
    let computerGPSV;

    // Function to calculate distance between two GPS coordinates (Haversine Formula)
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

                // Update Car GPS
                if (data.car_gps && data.car_gps.lat !== 0) {
                    carMarker = addOrUpdateMarker(carMarker, data.car_gps.lat, data.car_gps.lon, "R/C Car GPS");
                }

                // Update Drone GPS
                if (data.drone_gps && data.drone_gps.lat !== 0) {
                    droneMarker = addOrUpdateMarker(droneMarker, data.drone_gps.lat, data.drone_gps.lon, "Drone GPS");
                }

                // Calculate distances
                let carLat = data.car_gps?.lat || 0, carLon = data.car_gps?.lon || 0;
                let droneLat = data.drone_gps?.lat || 0, droneLon = data.drone_gps?.lon || 0;
                let computerLat = computerGPSV?.lat || 0, computerLon = computerGPSV?.lon || 0;
                if (carLat && carLon && droneLat && droneLon && computerLat && computerLon) {
                    let distanceComputerCar = haversineDistance(computerLat, computerLon, carLat, carLon);
                    let distanceCarDrone = haversineDistance(carLat, carLon, droneLat, droneLon);
                    let distanceDroneComputer = haversineDistance(droneLat, droneLon, computerLat, computerLon);

                    // Send distances to script
                    window.dispatchEvent(new CustomEvent("gpsLocationUpdate", {
                        detail: {
                            distanceComputerCar,
                            distanceCarDrone,
                            distanceDroneComputer
                        }
                    }));
                }
                else{
                    console.log("Missing a GPS location")
                    console.log("Car GPS:", { lat: carLat, lon: carLon });
                    console.log("Drone GPS:", { lat: droneLat, lon: droneLon });
                    console.log("Computer GPS:", { lat: computerLat, lon: computerLon });
                }
            })
            .catch(error => console.error("❌ Error fetching GPS data:", error));
    }

    // Fetch car GPS data every second
    setInterval(fetchAndUpdateGPS, 1000);

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
                computerGPSV = computerGPS;

                // Dispatch event with computer GPS
                //window.dispatchEvent(new CustomEvent("gpsLocationUpdate", {
                //    detail: { computer_gps: computerGPS }
                //}));
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
