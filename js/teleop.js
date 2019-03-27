// 1.82 and 0.26 are the max angular and linear velocities of the TurtleBot3.
var angularVel = 1.0/2;
var linearVel = 0.25/2;

document.addEventListener('keydown', function (event) {

    var cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });

    var xLinear = 0.0;
    var zAngular = 0.0;

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

    cmdVel.publish(twist);
}, true);

document.addEventListener('mouseup', function () {
    angularVel = (1.0 / 100) * document.getElementById("vel_slider").value;
    linearVel = (0.25 / 100) * document.getElementById("vel_slider").value;
});