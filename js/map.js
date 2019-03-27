// https://leafletjs.com/examples/quick-start/
// https://leafletjs.com/examples/crs-simple/crs-simple.html

var map = L.map('map', {
    crs: L.CRS.Simple
});

var meterPerPixel = 0.00668;

// Setting corner1 and corner2 empirically.
var corner1 = L.latLng(6.0, 0.0);
var corner2 = L.latLng(0.0, 5.0);
var bounds = L.latLngBounds(corner1, corner2);

var image = L.imageOverlay('img/map.png', bounds).addTo(map);

map.fitBounds(bounds);

// For calibration.
// var marker1 = L.marker([5.0, 1.5]).addTo(map);
// var marker2 = L.marker([1.0, 3.5]).addTo(map);

var popup = L.popup();

function onMapClick(e) {
    publishGoal(e.latlng);
    
    popup
        .setLatLng(e.latlng)
        .setContent("Robot will navigate to " + e.latlng.toString() + ".")
        .openOn(map);
}

map.on('click', onMapClick);