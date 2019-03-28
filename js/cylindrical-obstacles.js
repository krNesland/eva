var cylindricalObstaclesClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/cylindrical_obstacles',
    serviceType : 'eva_a/ReportCylindricalObstacle'
});

function callCylindricalObstacles() {

    console.log("Trying to call CylindricalObstacles.");
    var request = new ROSLIB.ServiceRequest();
    
    cylindricalObstaclesClient.callService(request, function(result) {
        if (result.success) {
            var latCenter = result.latCenter;
            var lngCenter = result.lngCenter;
            var radius = (result.radius)/meterPerPixel;

            var currMarker = L.circleMarker([latCenter, lngCenter], {
                color: 'red',
                fillColor: '#700',
                radius: radius
            }).addTo(map);
        }
        else {
            console.log("No cylindrical obstacles found.");
        }
    });
}