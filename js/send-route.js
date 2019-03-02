// Calling a service (make the robot follow a route)
// -----------------

var followRouteClient = new ROSLIB.Service({
    ros : ros,
    name : '/follow_route',
    serviceType : 'eva_a/FollowRoute'
});

var request = new ROSLIB.ServiceRequest({
    xVec : [2.5, 6.332, 2.564],
    yVec : [5.323, 1, -4.32]
});

followRouteClient.callService(request, function(result) {
    console.log('Result for service call on '
        + followRouteClient.name
        + ': '
        + result.success);
});