// Teleoperation of the robot by sending Twist messages.

// 1.82 and 0.26 are the max angular and linear velocities of the TurtleBot3.
var angularVel = 1.0/2;
var linearVel = 0.25/2;

// Every time a key is pressed.
document.addEventListener('keydown', function (event) {
    // Specifying the topic we want to publish on.
    var cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });

    var xLinear = 0.0;
    var zAngular = 0.0;

    // Determining which command to send based on which key is pressed.
    if (event.keyCode == 37) {
        zAngular = angularVel;
    }
    else if (event.keyCode == 39) {
        zAngular = -angularVel;
    }
    else if (event.keyCode == 38) {
        xLinear = linearVel;
    }
    else if (event.keyCode == 40) {
        xLinear = -linearVel;
    }

    // Creating a Twist message.
    var twist = new ROSLIB.Message({
        linear: {
            x: xLinear,
            y: 0.0,
            z: 0.0
        },
        angular: {
            x: 0.0,
            y: 0.0,
            z: zAngular
        }
    });

    // Publishing the message.
    cmdVel.publish(twist);
}, true);

// Reading the value of the slider every time the left mouse button is released and updating the velocities.
document.addEventListener('mouseup', function () {
    angularVel = (1.0 / 100) * document.getElementById("vel_slider").value;
    linearVel = (0.25 / 100) * document.getElementById("vel_slider").value;
});