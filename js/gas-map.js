// Building a heatmap using Leaflet.heat.

// Initializing an empty heatmap.
var heat = L.heatLayer([], {
    radius: 10,
    blur: 5, 
    maxZoom: 10,
    max: 1.0,
    minOpacity: 0.3,

    gradient: {
        0.0: 'green',
        0.5: 'yellow',
        0.8: 'red'
    }

}).addTo(map);

// A matrix that will make sure that the gas level is only stored once for every "cell" of the map.
var latLngArray = Array(100).fill().map(() => Array(100).fill(0));

// Every time a message is received from the robot.
gasListener.subscribe(function(message) {
    gasValue = message.data.toFixed(1);

    // Bottom left of array is defined to be at lat=-4 and lng=-3. The array has a resolution of 10 per lat/lng unit.
    var newLat10 = Math.floor(robotMarker.getLatLng().lat*10) + 40;
    var newLng10 = Math.floor(robotMarker.getLatLng().lng*10) + 30;

    if (newLat10 < 0) {
        console.log("Latitude out of bounds.")
    }
    else if (newLng10 < 0) {
        console.log("Longitude out of bounds.")
    }
    // A gas reading of 100 will generate 10 points. 30 will generate 3 and so on.
    else {
        // If not visited the cell before.
        if (latLngArray[newLat10][newLng10] < 1) {
            // Adding points to the heatmap. The number of points is determined by the concentration of the gas.
            for (var i = 0; i < Math.floor(gasValue/10); i++) {
                heat.addLatLng([(newLat10 - 40)/10.0 + i/200.0, (newLng10 - 30)/10.0]);
            }

            // Noting that this cell has now been visited. Will not add more point to this cell later.
            latLngArray[newLat10][newLng10] = 1;
        }
    }
});