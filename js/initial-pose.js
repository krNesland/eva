// First, we create a Topic object with details of the topic's name and message type.
var initialposeTopic = new ROSLIB.Topic({
    ros : ros,
    name : 'initialpose',
    messageType : 'geometry_msgs/PoseWithCovarianceStamped'
});

function initialPose() {
    var initialposeMsg = new ROSLIB.Message({
        header: {
            frame_id: "map"
        },
        pose: {
            pose: {
                position: {
                    x: 1.0,
                    y: -3.0,
                    z: 0.0
                },
                orientation: {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0
                }
            },
            covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        }
    });

    initialposeTopic.publish(initialposeMsg);

    console.log("Sent initialpose.");
}