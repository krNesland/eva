
var currWaypoint = 0;

var waypointListener = new ROSLIB.Topic({
    ros : ros,
    name : '/eva/curr_waypoint',
    messageType : 'std_msgs/UInt32'
});

waypointListener.subscribe(function(message) {
    currWaypoint = message.data;
    console.log(currWaypoint);
});

// First, we create a Topic object with details of the topic's name and message type.
var cancelTopic = new ROSLIB.Topic({
    ros : ros,
    name : 'move_base/cancel',
    messageType : 'actionlib_msgs/GoalID'
});

function cancelNav() {

    // Then we create the payload to be published. The object we pas in to ros.
    var cancelMsg = new ROSLIB.Message({
        id: 'follow_route_goal_' + currWaypoint
    });

    // And finally, publish.
    cancelTopic.publish(cancelMsg);

    console.log("Canceled goal: " + currWaypoint);
}

var followRouteClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/follow_route',
    serviceType : 'eva_a/FollowRoute'
});

var lats = [3.3, 5.3, 2.4, 2.4, 0.5];
var lngs = [2.9, 2.0, 0.6, 2.3, 2.9];

function callFollowRoute() {

    if (currWaypoint < lats.length) {
        console.log("Trying to call FollowRoute from waypoint " + currWaypoint + ".");
        var request = new ROSLIB.ServiceRequest({
            latVec : lats.slice(currWaypoint),
            lngVec : lngs.slice(currWaypoint)
        });
        
        followRouteClient.callService(request, function(result) {
            console.log('Result for service call on '
                + followRouteClient.name
                + ': '
                + result.success);
        });
    }
    else {
        console.log("All waypoints are already reached.")
    }
}