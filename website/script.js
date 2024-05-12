var gamepad_axis_prev = "null";
var gamepad_button_prev = "null";
var ip = "131.230.197.82";
var port = "9090";
var url_string = `ws://${ip}:${port}`;

// Connect to ROSBridge
var ros = new ROSLIB.Ros({
    url: url_string // Replace with your ROSBridge server IP
});

// Create a ROS topic
var connectionTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/connection_status', // Replace with your desired topic
    messageType: 'std_msgs/Byte' // Replace with the topic's message type
});


ros.on('connection', function() {
    console.log('Connected to ROSBridge!');
    // Axis data ros message
    var connectionStatus = new ROSLIB.Message({
        data: 0
    });

    setInterval(() => connectionTopic.publish(connectionStatus), 250);
});

ros.on('error', function(error) {
    console.log('Error connecting to ROSBridge:', error);
});

ros.on('close', function() {
    
    setTimeout(() => ros.connect(url_string), 5000);
});

// Create a ROS topic
var axisTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/gamepad_axis', // Replace with your desired topic
    messageType: 'std_msgs/Int8MultiArray' // Replace with the topic's message type
});

// Create a ROS topic
var buttonTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/gamepad_button', // Replace with your desired topic
    messageType: 'std_msgs/Int8MultiArray' // Replace with the topic's message type
});

// var diagnosticTopic = new ROSLIB.Topic({
//     ros: ros,
//     name: '/diagnostics',
//     messageType: 'diagnostic_msgs/DiagnosticArray'
// });

var imageTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/compressed_image', // Replace with your desired topic
    messageType: 'std_msgs/String' // Replace with the topic's message type
});

imageTopic.subscribe(function(message) {

    document.getElementById("video_out").src = "data:image/jpeg;base64," + message.data;

});

// Subscribe to the topic
document.getElementById('subscribe-button').onclick = function() {
    axisTopic.subscribe(function(message) {
        document.getElementById('axis-display').innerHTML = message.data;
    });
    buttonTopic.subscribe(function(message) {
        document.getElementById('button-display').innerHTML = message.data;
    });
    // diagnosticTopic.subscribe(function(message) {

    //     document.getElementById('diagnostic-display').innerHTML = message.status.map(status => {
    //         status.name + " " + status.message;
    //     });
    // });
};

// Controller polling using the Gamepad API
window.addEventListener("gamepadconnected", function(e) {
    console.log("Gamepad connected!");
    setInterval(readControllerData, 75); // Poll for updates
});


function readControllerData() {
    var gamepad = navigator.getGamepads()[0]; // Assuming the first connected gamepad
    
    if (gamepad == undefined) {
	    return;
	}

    if (!ros.isConnected) {
        return;
    }

    gamepad_axis = gamepad.axes.map(axis => parseInt(axis.toFixed(2)*100))
    gamepad_button = gamepad.buttons.map(button => button.value = parseInt((button.value.toFixed(2)*100)));

    // Axis data ros message
    var axisData = new ROSLIB.Message({
        data: gamepad_axis
    });

    axisTopic.publish(axisData);
    
    var buttonData = new ROSLIB.Message({
        data: gamepad_button
    });

    buttonTopic.publish(buttonData);
}
