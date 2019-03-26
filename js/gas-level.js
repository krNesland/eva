var gasDial = document.getElementById('gas-dial');
gasDial.innerHTML = '49';

var gasListener = new ROSLIB.Topic({
    ros : ros,
    name : '/eva/gas_level',
    messageType : 'std_msgs/String'
  });

gasListener.subscribe(function(message) {
    gasDial.innerHTML = message.data;
});