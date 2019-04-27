class Obstacle {
    constructor(latCenter, lngCenter) {
        this.latCenter = latCenter;
        this.lngCenter = lngCenter;
    }

    draw() {
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

var listener = new ROSLIB.Topic({
    ros: ros,
    name: '/eva/obstacles',
    messageType: 'eva_a/Obstacles'
});

listener.subscribe(function (message) {
    obstacleLayer.clearLayers();
    var obstacles = [];

    for (var i = 0; i < message.numObstacles; i++) {
        obstacles.push(new Obstacle(message.latCenters[i], message.lngCenters[i]));
    }

    for (var j = 0; j < message.numObstacles; j++) {
        obstacles[j].draw()
    }
});