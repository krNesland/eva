
var followRouteClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/follow_route',
    serviceType : 'eva_a/FollowRoute'
});

function callFollowRoute() {

    console.log("Trying to call FollowRoute.");
    var request = new ROSLIB.ServiceRequest({
        latVec : [3.3, 5.3, 2.4, 2.4, 0.5],
        lngVec : [2.9, 2.0, 0.6, 2.3, 2.9]
    });
    
    followRouteClient.callService(request, function(result) {
        console.log('Result for service call on '
            + followRouteClient.name
            + ': '
            + result.success);
    });
}