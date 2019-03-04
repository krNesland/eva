// Calling a service (make the robot follow a route)
// -----------------

var followRouteClient = new ROSLIB.Service({
    ros : ros,
    name : '/follow_route',
    serviceType : 'eva_a/FollowRoute'
});

function callFollowRoute() {

    console.log("Trying to call FollowRoute.");
    var request = new ROSLIB.ServiceRequest({
        latVec : [1.6, 0.85, 1.8, 0.5],
        lngVec : [0.8, 1.73, 4, 0.5]
    });
    
    followRouteClient.callService(request, function(result) {
        console.log('Result for service call on '
            + followRouteClient.name
            + ': '
            + result.success);
    });
}