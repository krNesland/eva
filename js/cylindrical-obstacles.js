var cylindricalObstaclesClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/cylindrical_obstacles',
    serviceType : 'eva_a/ReportCylindricalObstacle'
});

var obstacleFound = false;
var obstacleLat = -1.0;
var obstacleLng = -1.0;

function callCylindricalObstacles() {

    console.log("Trying to call CylindricalObstacles.");
    var request = new ROSLIB.ServiceRequest();
    
    cylindricalObstaclesClient.callService(request, function(result) {
        if (result.success) {
            var latCenter = result.latCenter;
            var lngCenter = result.lngCenter;
            var radius = (result.radius)*70

            obstacleFound = true;
            obstacleLat = latCenter;
            obstacleLng = lngCenter;

            var currMarker = L.circleMarker([latCenter, lngCenter], {
                color: 'red',
                fillColor: '#700',
                radius: radius
            }).addTo(map);

            console.log("Cylinder of radius " + radius + " found at ("+ latCenter + "," + lngCenter + ")");
        }
        else {
            console.log("No cylindrical obstacles found.");
        }
    });
}