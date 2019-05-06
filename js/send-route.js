
var followRouteClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/follow_route',
    serviceType : 'eva_a/FollowRoute'
});

function callFollowRoute() {

    console.log("Trying to call FollowRoute.");
    var request = new ROSLIB.ServiceRequest({
        latVec: [-2.2, -1.3, -2.1, 0.2, 2.4, 0.0],
        lngVec: [-0.3, 2.9, 4.3, 5.4, -1.0, 0.0],
        startFrom: 0
    });
    
    followRouteClient.callService(request, function(result) {
        console.log('Result for service call on '
            + followRouteClient.name
            + ': '
            + result.success);
    });
}