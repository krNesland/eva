var driveAroundClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/drive_around',
    serviceType : 'eva_a/DriveAround'
});

function callDriveAround(obstaclePos, radius) {

    console.log("Trying to call DriveAround.");
    var request = new ROSLIB.ServiceRequest({
        obstaclePosX: obstaclePos.lat,
        obstaclePosY: -obstaclePos.lng,
        circlingRadius: radius
    });
    
    driveAroundClient.callService(request, function(result) {
        if (result.area) {
            console.log("Area: " + result.area);
        }
        else {
            console.log("Was not able to drive around.");
        }
    });
}