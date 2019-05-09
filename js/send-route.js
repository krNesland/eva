
var followRouteClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/follow_route',
    serviceType : 'eva_a/FollowRoute'
});

function callFollowRoute() {

    console.log("Trying to call FollowRoute.");
    var request = new ROSLIB.ServiceRequest({
        latVec: [-2.3, -0.8, 0.0, 0.25, -2.35, -1.0, 2.4, 0.0],
        lngVec: [1.0, 2.75, 0.0, 3.5, 4.0, 4.8, -0.5, 0.0],
        startFrom: 0
    });
    
    followRouteClient.callService(request, function(result) {
        console.log('Result for service call on '
            + followRouteClient.name
            + ': '
            + result.success);
    });
}