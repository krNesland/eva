// https://leafletjs.com/examples/quick-start/
// https://leafletjs.com/examples/crs-simple/crs-simple.html

var map = L.map('map', {
    crs: L.CRS.Simple
});

var meterPerPixel = 0.00668;

// Setting corner1 and corner2 empirically.
var corner1 = L.latLng(2.6, -0.39);
var corner2 = L.latLng(-0.35, 5.6);
var bounds = L.latLngBounds(corner1, corner2);

var image = L.imageOverlay('img/bitmap.png', bounds).addTo(map);

map.fitBounds(bounds);

var marker = L.marker([1.5, 1.5]).addTo(map);

// For calibration.
// var marker1 = L.marker([0.0, 0.0]).addTo(map);
// var marker2 = L.marker([2.07, 5.05]).addTo(map);

var popup = L.popup();

function onMapClick(e) {
    publishGoal(e.latlng);
    
    popup
        .setLatLng(e.latlng)
        .setContent("You clicked the map at " + e.latlng.toString() + ".")
        .openOn(map);
}

map.on('click', onMapClick);