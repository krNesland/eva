var takePictureClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/take_picture',
    serviceType : 'eva_a/TakePicture'
});

function callTakePicture(obstaclePos) {

    console.log("Trying to call TakePicture.");
    var request = new ROSLIB.ServiceRequest({
        obstaclePosX: obstaclePos.lat,
        obstaclePosY: -obstaclePos.lng
    });
    
    takePictureClient.callService(request, function(result) {
        if (result.success) {
            console.log("Picture taken.");

            // Small trick to reload the image.
            document.getElementById("obstacle-img").src="../img/captures/capture.jpg?random="+new Date().getTime();

            var elems = document.querySelectorAll('.modal');
            var instances = M.Modal.init(elems);

            instances[0].open()
        }
        else {
            console.log("Was not able to take a picture.");
        }
    });
}