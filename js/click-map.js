// Sending navigation goal to the move_base action server when the map is clicked.

// Specifying the action server.
var navigationClient = new ROSLIB.ActionClient({
    ros : ros,
    serverName : '/move_base',
    actionName : 'move_base_msgs/MoveBaseAction'
});

// What happens when the map is clicked.
function publishGoal(latlng) {

    // Putting together the pose.
    var positionVec3 = new ROSLIB.Vector3({x:latlng.lng, y:latlng.lat, z:0});
    var orientation = new ROSLIB.Quaternion({x:0, y:0, z:0, w:1.0});
    var pose = new ROSLIB.Pose({
        position : positionVec3,
        orientation : orientation
    });

    // Specifying the goal.
    var goal = new ROSLIB.Goal({
        actionClient : navigationClient,
        goalMessage : {
            target_pose : {
                header : {
                    frame_id : '/map'
                },
                pose : pose
            }
        }
    });

    // What happens when the move_base server returns a result.
    goal.on('result', function(result) {
        console.log("Goal reached or cancelled.");
    });

    // Sending the goal.
    goal.send();
    console.log("Sent message to navigate to (" + latlng.lat + ", " + latlng.lng + ").");
}

function onMapClick(e) {
    publishGoal(e.latlng);
}

map.on('click', onMapClick);