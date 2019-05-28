// Functionality related to obstacles.

// A class that stores info about an obstacle.
class Obstacle {
    constructor(latCenter, lngCenter) {
        this.latCenter = latCenter;
        this.lngCenter = lngCenter;
    }

    draw() {
        // Adding a marker on the map and specifying which function to run on obstacle click.
        L.circleMarker([this.latCenter, this.lngCenter], {
            color: 'red',
            fillColor: '#700'
        }).addTo(obstacleLayer).on('click', function(e) {
            if (document.getElementById("drive-around").checked) {
                callDriveAround(e.latlng, 0.6);
            }
            else {
                callTakePicture(e.latlng);
            }
        });
    }
}

// Specifying the topic we want to listen to.
var obstacleListener = new ROSLIB.Topic({
    ros: ros,
    name: '/eva/obstacles',
    messageType: 'eva_a/Obstacles'
});

// Every time an obstacle message is received.
obstacleListener.subscribe(function (message) {
    // Removing all the obstacles currently drawn on the map.
    obstacleLayer.clearLayers();
    
    var obstacles = [];

    // Filling the list with obstacles.
    for (var i = 0; i < message.numObstacles; i++) {
        obstacles.push(new Obstacle(message.latCenters[i], message.lngCenters[i]));
    }

    // Drawing the obstacles.
    for (var j = 0; j < message.numObstacles; j++) {
        obstacles[j].draw()
    }
});