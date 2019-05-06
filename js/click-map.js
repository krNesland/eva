
// First, we create a Topic object with details of the topic's name and message type.
var navTopic = new ROSLIB.Topic({
    ros : ros,
    name : 'move_base/goal',
    messageType : 'move_base_msgs/MoveBaseActionGoal'
});

var seqId = 0;

// At click of map.
function publishGoal(latlng) {

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
                        x: latlng.lng,
                        y: latlng.lat,
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
        },
        goal_id: {
            id: 'clickMapGoal_' + seqId
        }
    });

    // And finally, publish.
    navTopic.publish(navMsg);
    seqId = seqId + 1;

    console.log("Sent message to navigate to (" + latlng.lat + ", " + latlng.lng + ").");
}

function onMapClick(e) {
    publishGoal(e.latlng);
}

map.on('click', onMapClick);