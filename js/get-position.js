var tfClient = new ROSLIB.TFClient({
    ros : ros,
    fixedFrame : 'map',
    angularThres : 0.01,
    transThres : 0.01
});

console.log('Waiting for transforms.');

tfClient.subscribe('base_footprint', function(tf) {
    marker.setLatLng(L.latLng(tf.translation.x, -tf.translation.y)); 
});