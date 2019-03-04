// Pre-defined position (at click of button).
function publishSetGoal() {

    var newLng = 1.0;
    var newLat = 1.0;

    // Then we create the payload to be published. The object we pas in to ros.
    var navMsg = new ROSLIB.Message({
        goal: {
            target_pose: {
                header: {
                    frame_id: "map"
                },
                pose: {
                    position: {
                        // Have to do some trickery with the axis.
                        x: newLat,
                        y: -newLng,
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

    console.log("Sent message to navigate to (" + newLat + ", " + newLng + ").");
}