
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

// Only want to update a position one time. Zero for un-visited "cells".
var latLngArray = Array(100).fill().map(() => Array(100).fill(0));

gasListener.subscribe(function(message) {
    gasValue = parseFloat(message.data);

    var newLat10 = Math.floor(robotMarker.getLatLng().lat*10);
    var newLng10 = Math.floor(robotMarker.getLatLng().lng*10);

    if (newLat10 < 0) {
        console.log("Latitude out of bounds.")
    }
    else if (newLng10 < 0) {
        console.log("Longitude out of bounds.")
    }
    // A gas reading of 100 will generate 10 points. 30 will generate 3 and so on.
    else {
        if (latLngArray[newLat10][newLng10] < 1) {
            for (var i = 0; i < Math.floor(gasValue/10); i++) {
                heat.addLatLng([newLat10/10.0 + i/200.0, newLng10/10.0]);
            }

            // Noting that this cell has now been visited.
            latLngArray[newLat10][newLng10] = 1;
        }
    }

});