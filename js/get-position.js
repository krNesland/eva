var tfClient = new ROSLIB.TFClient({
    ros : ros,
    fixedFrame : 'map',
    angularThres : 0.01,
    transThres : 0.01
});

console.log('Waiting for transforms.');
/*
A simpler marker.
var currMarker = L.circleMarker([1.5, 1.5], {
    color: 'black',
    fillColor: '#777'
}).addTo(map);
*/


var taurobIcon = L.icon({
    iconUrl: 'img/taurob_marker.png',
    shadowUrl: 'img/taurob_marker_shadow.png',

    iconSize:     [30, 29], // size of the icon
    shadowSize:   [32, 31], // size of the shadow
    iconAnchor:   [15, 15], // point of the icon which will correspond to marker's location
    shadowAnchor: [15, 15],  // the same for the shadow
    popupAnchor:  [-3, -76] // point from which the popup should open relative to the iconAnchor
});

L.marker([1.5, 1.5], {icon: taurobIcon}).addTo(map);

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