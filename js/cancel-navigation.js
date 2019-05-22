// Either following or teleoperating.
var following = 0;
var currWaypoint = 0;

var followRouteClient = new ROSLIB.ActionClient({
    ros : ros,
    serverName : '/follow_route_server',
    actionName : '/eva_a/FollowRouteAction'
});

var goal = new ROSLIB.Goal({
    actionClient : followRouteClient,
    goalMessage : {
        latVec: [-2.3, -0.9, 0.0, 0.25, -2.35, -1.0, 2.4, 0.0],
        lngVec: [1.0, 2.65, 0.0, 3.5, 4.0, 4.8, -0.5, 0.0],
        firstWaypoint: 0
    }
});

//header.seq header.stamp header.frame_id goal_id.stamp goal_id.id goal.latVec goal.lngVec goal.firstWaypoint

goal.on('feedback', function(feedback) {
    console.log('Feedback: ' + feedback.headingFor);
    currWaypoint = feedback.headingFor;
});

goal.on('result', function(result) {
    if (result.success == 1) {
        console.log("Successfully completed the route.");
        currWaypoint = 0;

        following = 0;
        document.getElementById("following-btn").innerText = "follow";
        document.getElementById("following-btn").classList.remove("red");
        document.getElementById("following-btn").classList.remove("darken-4");
    }
    else {
        console.log("Route was cancelled on the way to waypoint: " + currWaypoint + ".");
    }
});

function callFollowRoute() {
    console.log("Trying to call FollowRoute.");
    goal.goalMessage.goal.firstWaypoint = currWaypoint;
    goal.send();
    console.log("Goal is sent.");
}

function cancelNav() {
    goal.cancel();
    console.log("Canceled goal: " + currWaypoint);
}

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