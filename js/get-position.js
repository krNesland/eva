var tfClient = new ROSLIB.TFClient({
    ros : ros,
    fixedFrame : 'map',
    angularThres : 0.01,
    transThres : 0.01
});

console.log('Waiting for transforms.');
var currMarker = L.marker([1.5, 1.5]).addTo(map);

tfClient.subscribe('base_footprint', function(tf) {
    currMarker.setLatLng(L.latLng(tf.translation.x, -tf.translation.y));
});

// Subscribing to a Topic (for visualizing the current goal).
// ----------------------

var goalListener = new ROSLIB.Topic({
    ros : ros,
    name : 'move_base/goal',
    messageType : 'move_base_msgs/MoveBaseActionGoal'
});

var goalLat = 0.0;
var goalLng = 0.0;
var goalMarker = L.marker([goalLat, goalLng]).addTo(map);


goalListener.subscribe(function(message) {
    goalLat = message.goal.target_pose.pose.position.x
    goalLng = - message.goal.target_pose.pose.position.y
    
    goalMarker.setLatLng(L.latLng(goalLat, goalLng));
});