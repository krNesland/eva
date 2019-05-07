
// Either following or teleoperating.
var following = 0;

function toggleFollowing() {
    if (following) {
        following = 0;
        document.getElementById("following-btn").innerText = "follow";
        document.getElementById("following-btn").classList.remove("red");
        document.getElementById("following-btn").classList.remove("darken-4");
        cancelNav();
    }
    else {
        following = 1;
        document.getElementById("following-btn").innerText = "teleoperate";
        document.getElementById("following-btn").classList.add("red");
        document.getElementById("following-btn").classList.add("darken-4");
        callFollowRoute();
    }
}

var currWaypoint = 0;
var callNr = -1;

// Listening to the current waypoint.

var waypointListener = new ROSLIB.Topic({
    ros : ros,
    name : '/eva/curr_waypoint',
    messageType : 'std_msgs/UInt32'
  });

  waypointListener.subscribe(function(message) {
    currWaypoint = message.data;
  });

// Cancelling the route following.

var cancelTopic = new ROSLIB.Topic({
    ros : ros,
    name : 'move_base/cancel',
    messageType : 'actionlib_msgs/GoalID'
});

function cancelNav() {

    var cancelMsg = new ROSLIB.Message({
        id: 'follow_route_goal_' + currWaypoint + '_' + callNr
    });

    cancelTopic.publish(cancelMsg);

    console.log("Canceled goal: " + currWaypoint);
}

// Calling the route following.

var followRouteClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/follow_route',
    serviceType : 'eva_a/FollowRoute'
});

var lats = [-2.3, 1.45, -0.65, 0.25, 2.35, 0.95, 2.35, 0.0];
var lngs = [-0.4, 2.65, 0.35, 2.9, 3.8, 4.85, 0.7, 0.0];

function callFollowRoute() {
    callNr = callNr + 1;

    if (currWaypoint < lats.length) {
        console.log("Trying to call FollowRoute from waypoint " + currWaypoint + ".");
        var request = new ROSLIB.ServiceRequest({
            latVec : lats,
            lngVec : lngs,
            startFrom: currWaypoint
        });
        
        followRouteClient.callService(request, function(result) {
            if (result.currWaypoint == 100) {
                console.log("Successfully completed the route.");
                currWaypoint = 0;

                following = 0;
                document.getElementById("following-btn").innerText = "follow";
                document.getElementById("following-btn").classList.remove("red");
                document.getElementById("following-btn").classList.remove("darken-4");
            }
            else {
                console.log("Route was cancelled on the way to waypoint: " + result.currWaypoint + ".");
                currWaypoint = result.currWaypoint;
            }
        });
    }
    else {
        console.log("All waypoints have already been reached.")
    }
}