var tfClient = new ROSLIB.TFClient({
    ros : ros,
    fixedFrame : 'map',
    angularThres : 0.01,
    transThres : 0.01
});

console.log('Waiting for transforms.');
/*
A simpler marker.
var robotMarker = L.circleMarker([1.5, 1.5], {
    color: 'black',
    fillColor: '#777'
}).addTo(map);
*/


var robotIcon = L.icon({
    iconUrl: '../img/taurob_marker.png',
    shadowUrl: '../img/taurob_marker_shadow.png',

    iconSize:     [20, 19], // size of the icon
    shadowSize:   [22, 21], // size of the shadow
    iconAnchor:   [11, 11], // point of the icon which will correspond to marker's location
    shadowAnchor: [11, 11],  // the same for the shadow
    popupAnchor:  [-3, -76] // point from which the popup should open relative to the iconAnchor
});

var robotMarker = L.marker([1.5, 1.5], {icon: robotIcon}).addTo(map);

tfClient.subscribe('base_footprint', function(tf) {
    robotMarker.setLatLng(L.latLng(tf.translation.x, -tf.translation.y));
});

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