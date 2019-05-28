// Allows the user to pause the inspection route, perform teleoperation and then continue the inspection route when wanted.

// Either following or teleoperating.
var following = 0;

// First waypoint is number zero.
var currWaypoint = 0;

// Specifying the action server we want to communicate with.
var followRouteClient = new ROSLIB.ActionClient({
    ros : ros,
    serverName : '/follow_route_server',
    actionName : '/eva_a/FollowRouteAction'
});

// Specifying the goal to be sent to the server.
var goal = new ROSLIB.Goal({
    actionClient : followRouteClient,
    goalMessage : {
        latVec: [-2.3, -0.9, 0.0, 0.25, -2.35, -1.0, 2.4, 0.0],
        lngVec: [1.0, 2.65, 0.0, 3.5, 4.0, 4.8, -0.5, 0.0],
        firstWaypoint: 0
    }
});

// What should happen when the action server returns feedback.
goal.on('feedback', function(feedback) {
    console.log('Feedback: ' + feedback.headingFor);
    currWaypoint = feedback.headingFor;
});

// What should happen when the action server returns the final result.
goal.on('result', function(result) {
    if (result.success == 1) {
        console.log("Successfully completed the route.");
        
        // Resetting the waypoint.
        currWaypoint = 0;

        // No longer following the inspection route.
        following = 0;

        // Resetting the button.
        document.getElementById("following-btn").innerText = "follow";
        document.getElementById("following-btn").classList.remove("red");
        document.getElementById("following-btn").classList.remove("darken-4");
    }
    else {
        console.log("Route was cancelled on the way to waypoint: " + currWaypoint + ".");
    }
});

// Sending the goal to the action server.
function callFollowRoute() {
    console.log("Trying to call FollowRoute.");
    goal.goalMessage.goal.firstWaypoint = currWaypoint;
    goal.send();
    console.log("Goal is sent.");
}

// Cancelling the route following.
function cancelNav() {
    goal.cancel();
    console.log("Canceled goal: " + currWaypoint);
}

// What happens when the button is pressed.
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