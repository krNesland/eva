// Ordering the robot to take a picture of an obstacle.

// Specifying the server we want to communicate with.
var takePictureClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/take_picture',
    serviceType : 'eva_a/TakePicture'
});

// Calling the server.
function callTakePicture(obstaclePos) {

    console.log("Trying to call TakePicture.");

    // Specifying the request.
    var request = new ROSLIB.ServiceRequest({
        obstaclePosX: obstaclePos.lng,
        obstaclePosY: obstaclePos.lat
    });
    
    // Displaying the recently captured image when the server is finished.
    takePictureClient.callService(request, function(result) {
        if (result.success) {
            console.log("Picture taken.");

            // Small trick to reload the image.
            document.getElementById("obstacle-img").src="../img/captures/capture.jpg?random="+new Date().getTime();

            // Opening a modal with the image.
            var elems = document.querySelectorAll('.modal');
            var instances = M.Modal.init(elems);
            instances[0].open()
        }
        else {
            console.log("Was not able to take a picture.");
        }
    });
}