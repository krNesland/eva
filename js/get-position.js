// Visualizing the position of the robot on the map.

// Specifying the connection to tf.
var tfClient = new ROSLIB.TFClient({
    ros : ros,
    fixedFrame : 'map',
    angularThres : 0.01,
    transThres : 0.01
});

console.log('Waiting for transforms.');

// Details about the marker.
var robotIcon = L.icon({
    iconUrl: '../img/taurob_marker.png',
    shadowUrl: '../img/taurob_marker_shadow.png',

    iconSize:     [20, 19], // size of the icon
    shadowSize:   [22, 21], // size of the shadow
    iconAnchor:   [11, 11], // point of the icon which will correspond to marker's location
    shadowAnchor: [11, 11],  // the same for the shadow
    popupAnchor:  [-3, -76] // point from which the popup should open relative to the iconAnchor
});

// Adding the marker to the map.
var robotMarker = L.marker([1.5, 1.5], {icon: robotIcon}).addTo(map);

// Subscribing to the pose of the robot and updating the marker when a new pose is received.
tfClient.subscribe('base_footprint', function(tf) {
    robotMarker.setLatLng(L.latLng(tf.translation.y, tf.translation.x));
});

// Specifying a subscription to the current navigation goal.
var goalListener = new ROSLIB.Topic({
    ros : ros,
    name : 'move_base/goal',
    messageType : 'move_base_msgs/MoveBaseActionGoal'
});

var goalLat = 0.0;
var goalLng = 0.0;

// Adding a goal marker on the map.
var goalMarker = L.marker([goalLat, goalLng]).addTo(map);

// Updating the goal marker position when a new goal is received.
goalListener.subscribe(function(message) {
    goalLat = message.goal.target_pose.pose.position.y;
    goalLng = message.goal.target_pose.pose.position.x;
    
    goalMarker.setLatLng(L.latLng(goalLat, goalLng));
});