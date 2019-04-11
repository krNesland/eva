class Obstacle {
    constructor(latCenter, lngCenter, radius) {
        this.latCenter = latCenter;
        this.lngCenter = lngCenter;
        this.radius = radius*70;
    }

    draw() {
        L.circleMarker([this.latCenter, this.lngCenter], {
            color: 'red',
            fillColor: '#700',
            radius: this.radius
        }).addTo(obstacleLayer);
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
        obstacles.push(new Obstacle(message.latCenters[i], message.lngCenters[i], message.radii[i]));
    }

    for (var j = 0; j < message.numObstacles; j++) {
        obstacles[j].draw()
    }
});