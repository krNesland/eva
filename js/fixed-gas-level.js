var fixedGasDial = document.getElementById('fixed-gas-dial');
fixedGasDial.innerHTML = '49';

var fixedGasListener = new ROSLIB.Topic({
    ros : ros,
    name : '/eva/fixed_gas_level',
    messageType : 'std_msgs/String'
  });

fixedGasListener.subscribe(function(message) {
    fixedGasDial.innerHTML = message.data;
});