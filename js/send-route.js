var followRouteClient = new ROSLIB.ActionClient({
    ros : ros,
    serverName : '/follow_route_server',
    actionName : '/eva_a/FollowRouteAction'
});

var goal = new ROSLIB.Goal({
    actionClient : followRouteClient,
    goalMessage : {
        latVec: [-2.3, -0.8, 0.0, 0.25, -2.35, -1.0, 2.4, 0.0],
        lngVec: [1.0, 2.75, 0.0, 3.5, 4.0, 4.8, -0.5, 0.0],
        firstWaypoint: 0
    }
});

//header.seq header.stamp header.frame_id goal_id.stamp goal_id.id goal.latVec goal.lngVec goal.firstWaypoint

console.log("Message is ready.");

goal.on('feedback', function(feedback) {
    console.log('Feedback: ' + feedback.headingFor);
});

goal.on('result', function(result) {
    console.log('Final Result: ' + result.success);
});

function callFollowRoute() {
    console.log("Trying to call FollowRoute.");
    goal.send();
    console.log("Goal is sent.");
}