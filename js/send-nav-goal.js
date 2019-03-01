// Publishing a Topic (setting the goal is actually a topic, not a service)
// ------------------

// First, we create a Topic object with details of the topic's name and message type.
var navTopic = new ROSLIB.Topic({
    ros : ros,
    name : 'move_base/goal',
    messageType : 'move_base_msgs/MoveBaseActionGoal'
});

function publishGoal(latlng) {
    console.log("x: " + latlng.lng);
    console.log("y: " + latlng.lat);

    // Then we create the payload to be published. The object we pas in to ros.
    var navMsg = new ROSLIB.Message({
        goal: {
            target_pose: {
                header: {
                    frame_id: "map"
                },
                pose: {
                    position: {
                        // Have to do some trickery with the axis.
                        x: latlng.lat,
                        y: -latlng.lng,
                        z: 0.0
                    },
                    orientation: {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0
                    }
                }
            }
        }
    });

    // And finally, publish.
    navTopic.publish(navMsg);
}

/*

// Subscribing to a Topic
// ----------------------

var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/rgb/image_raw',
    messageType : 'sensor_msgs/Image'
});

const canvas = document.getElementById('camera');
const ctx = canvas.getContext('2d');
const arr = new Uint8ClampedArray(4*640*480);

listener.subscribe(function(message) {
    for (var i = 0; i < arr.length; i+=4) {
        arr[i + 0] = 2*message.data.charCodeAt(i + 0);
        arr[i + 1] = 2*message.data.charCodeAt(i + 1);
        arr[i + 2] = 2*message.data.charCodeAt(i + 2);
        arr[i + 3] = 255;
    }

    console.log(i);

    var imageData = new ImageData(arr, 640, 480);
    ctx.putImageData(imageData, 0, 0);

    console.log(message.data.length);
    console.log(message.data);

    console.log(message.encoding);

    listener.unsubscribe();
});
*/