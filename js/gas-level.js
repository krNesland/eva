// Same as fixed-gas-level, but for the sensor "attached" to the robot.

var gasDial = document.getElementById('gas-dial');
gasDial.innerHTML = '49';

var gasListener = new ROSLIB.Topic({
    ros : ros,
    name : '/eva/gas_level',
    messageType : 'std_msgs/Float32'
  });

gasListener.subscribe(function(message) {
    gasDial.innerHTML = message.data.toFixed(1).toString();
});