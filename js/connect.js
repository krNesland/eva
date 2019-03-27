// Connecting to ROS
// -----------------
var ros = new ROSLIB.Ros();

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
    console.log(error);
    document.getElementById("connection-symbol").style.color = "#80d8ff";
});
// Find out exactly when we made a connection.
ros.on('connection', function() {
    console.log('Connection made!');
    document.getElementById("connection-symbol").style.color = "white";
});
ros.on('close', function() {
    console.log('Connection closed.');
    document.getElementById("connection-symbol").style.color = "#80d8ff";
});

// Create a connection to the rosbridge WebSocket server.
ros.connect('ws://localhost:9090')

