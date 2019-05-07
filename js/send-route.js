
var followRouteClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/follow_route',
    serviceType : 'eva_a/FollowRoute'
});

function callFollowRoute() {

    console.log("Trying to call FollowRoute.");
    var request = new ROSLIB.ServiceRequest({
        latVec: [-2.3, 1.45, -0.65, 0.25, 2.35, 0.95, 2.35, 0.0],
        lngVec: [-0.4, 2.65, 0.35, 2.9, 3.8, 4.85, 0.7, 0.0],
        startFrom: 0
    });
    
    followRouteClient.callService(request, function(result) {
        console.log('Result for service call on '
            + followRouteClient.name
            + ': '
            + result.success);
    });
}