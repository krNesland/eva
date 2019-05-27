var fixedGasDial = document.getElementById('fixed-gas-dial');
fixedGasDial.innerHTML = '49';

var fixedGasListener = new ROSLIB.Topic({
    ros : ros,
    name : '/eva/fixed_gas_level',
    messageType : 'std_msgs/Float32'
  });

fixedGasListener.subscribe(function(message) {
    fixedGasDial.innerHTML = message.data.toFixed(1).toString();
});