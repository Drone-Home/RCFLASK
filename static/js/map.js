document.addEventListener('DOMContentLoaded', function () {
    console.log("✅ map.js loaded successfully");

    // Initialize the map
    const map = L.map('map').setView([0, 0], 15); // Default center

    // Add a satellite tile layer
    L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
        attribution: '© Esri, Maxar, Earthstar Geographics, and the GIS User Community'
    }).addTo(map);

    let carMarker, droneMarker, computerMarker;

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

                // Dispatch event for other scripts
                window.dispatchEvent(new CustomEvent("gpsLocationUpdate", { detail: data }));
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
