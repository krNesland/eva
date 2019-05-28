// Ordering the robot to drive around an obstacle.

// Specifying the server.
var driveAroundClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/drive_around',
    serviceType : 'eva_a/DriveAround'
});

function callDriveAround(obstaclePos, radius) {
    console.log("Trying to call DriveAround.");

    // Putting together the request.
    var request = new ROSLIB.ServiceRequest({
        obstaclePosX: obstaclePos.lng,
        obstaclePosY: obstaclePos.lat,
        circlingRadius: radius
    });
    
    // Sending the request.
    driveAroundClient.callService(request, function(result) {
        
        // If the robot succesfully has driven around the obstacle.
        if (result.success) {
            var latlngs = [];

            // Filling a list of obstacle vertices.
            for (var i = 0; i < result.lats.length; i++) {
                latlngs.push([result.lats[i], result.lngs[i]]);
            }

            // Drawing a polygon on the map where the obstacle is believed to be.
            var polygon = L.polygon(latlngs, {color: 'green'}).addTo(map);
        }
        else {
            console.log("Was not able to investigate obstacle more precisely.");
        }
    });
}