// Updating the dial showing the gas level measured by the fixed, wall-mounted, sensor.

var fixedGasDial = document.getElementById('fixed-gas-dial');
fixedGasDial.innerHTML = '49';

// Specifying the topic to listen to.
var fixedGasListener = new ROSLIB.Topic({
    ros : ros,
    name : '/eva/fixed_gas_level',
    messageType : 'std_msgs/Float32'
  });

// Updating when a message is received.
fixedGasListener.subscribe(function(message) {
    fixedGasDial.innerHTML = message.data.toFixed(1).toString();
});