// https://leafletjs.com/examples/quick-start/
// https://leafletjs.com/examples/crs-simple/crs-simple.html

// Setting up the map using an image. Geo-referencing.

var map = L.map('map', {
    crs: L.CRS.Simple
});

// Setting corner1 and corner2 empirically. LatLng of top left and bottom right of image.
var corner1 = L.latLng(2.85, -1.5);
var corner2 = L.latLng(-2.7, 6.0);
var bounds = L.latLngBounds(corner1, corner2);

// Specifying the image file.
var image = L.imageOverlay('../img/leaflet_map.png', bounds).addTo(map);

// Geo-referencing. Making sure that it matches the object frame.
map.fitBounds(bounds);

// Adding a layer on top of the map that will be used to mark obstacles.
var obstacleLayer = L.layerGroup().addTo(map);

// For calibration.
//var marker1 = L.marker([0.0, -0.6]).addTo(map);
//var marker2 = L.marker([1.0, 3.5]).addTo(map);