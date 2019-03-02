// Publishing a Topic (setting the goal is actually a topic, not a service)
// ------------------

// First, we create a Topic object with details of the topic's name and message type.
var navTopic = new ROSLIB.Topic({
    ros : ros,
    name : 'move_base/goal',
    messageType : 'move_base_msgs/MoveBaseActionGoal'
});

// At click of map.
function publishGoal(latlng) {
    console.log("x: " + latlng.lng);
    console.log("y: " + latlng.lat);

    // Then we create the payload to be published. The object we pas in to ros.
    var navMsg = new ROSLIB.Message({
        goal: {
            target_pose: {
                header: {
                    frame_id: "odom"
                },
                pose: {
                    position: {
                        // Have to do some trickery with the axis.
                        x: latlng.lat,
                        y: -latlng.lng,
                        z: 0.0
                    },
                    orientation: {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0
                    }
                }
            }
        }
    });

    // And finally, publish.
    navTopic.publish(navMsg);
}

// Pre-defined position (at click of button).
function publishSetGoal() {
    console.log("Moving to pre-defined position.");

    // Then we create the payload to be published. The object we pas in to ros.
    var navMsg = new ROSLIB.Message({
        goal: {
            target_pose: {
                header: {
                    frame_id: "odom"
                },
                pose: {
                    position: {
                        // Have to do some trickery with the axis.
                        x: 1.0,
                        y: -1.0,
                        z: 0.0
                    },
                    orientation: {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0
                    }
                }
            }
        }
    });

    // And finally, publish.
    navTopic.publish(navMsg);
}