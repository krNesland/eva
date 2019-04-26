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
        if (result.success) {
            var latlngs = [];

            for (var i = 0; i < result.lats.length; i++) {
                latlngs.push([result.lats[i], result.lngs[i]]);
            }

            var polygon = L.polygon(latlngs, {color: 'green'}).addTo(map);
        }
        else {
            console.log("Was not able to investigate obstacle more precisely.");
        }
    });
}