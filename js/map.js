// https://leafletjs.com/examples/quick-start/
// https://leafletjs.com/examples/crs-simple/crs-simple.html

var map = L.map('map', {
    crs: L.CRS.Simple
});

var marker = L.marker([0.0, 0.0]).addTo(map);

var meterPerPixel = 0.00668;

var bottomLeft = [435*meterPerPixel, -59*meterPerPixel];
var topRight = [-53*meterPerPixel, 830*meterPerPixel];

var bounds = [bottomLeft, topRight];
var image = L.imageOverlay('img/bitmap.png', bounds).addTo(map);

map.fitBounds(bounds);

var popup = L.popup();

function onMapClick(e) {
    publishGoal(e.latlng);
    
    popup
        .setLatLng(e.latlng)
        .setContent("You clicked the map at " + e.latlng.toString())
        .openOn(map);
}

map.on('click', onMapClick);