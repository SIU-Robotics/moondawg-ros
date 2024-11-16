// Parameters - change these to match the Pi
var ip = "131.230.197.82"; // IP of the Pi
var port = "9090"; // Port of webserver node

var gamepad_axis_prev = "null";
var gamepad_button_prev = "null";
var url_string = `ws://${ip}:${port}`;

// Connect to and set up ros bridge
var ros = new ROSLIB.Ros({
    url: url_string
});

ros.on('connection', function() {
    console.log('Connected to ROSBridge!');
    
    var connectionStatus = new ROSLIB.Message({
        data: 0
    });

    setInterval(() => connectionTopic.publish(connectionStatus), 250);
});

ros.on('error', function(error) {
    console.log('Error connecting to ROSBridge:', error);
});

ros.on('close', function() {
    setTimeout(() => ros.connect(url_string), 5000); // Retry connection every 5000 ms
});

// Define topics
var connectionTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/connection_status',
    messageType: 'std_msgs/Byte'
});
var axisTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/controller_parser/gamepad_axis',
    messageType: 'std_msgs/Int8MultiArray'
});
var buttonTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/controller_parser/gamepad_button',
    messageType: 'std_msgs/Int8MultiArray'
});
var imageTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/controller_parser/compressed_image',
    messageType: 'std_msgs/String'
});

// Parameters
ros.getParams(function(params) {
    document.getElementById('params-display').innerHTML = params;
});

// Subscribe to topics
imageTopic.subscribe(function(message) {
    document.getElementById("video_out").src = "data:image/jpeg;base64," + message.data;
});
axisTopic.subscribe(function(message) {
    document.getElementById('axis-display').innerHTML = message.data;
});
buttonTopic.subscribe(function(message) {
    document.getElementById('button-display').innerHTML = message.data;
});

// Connect gamepad
window.addEventListener("gamepadconnected", function(e) {
    console.log("Gamepad connected!");
    setInterval(readControllerData, 75); // Read from controller every 75 ms
});

// Function to be called every time the controlled is read from
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

    var axisData = new ROSLIB.Message({
        data: gamepad_axis
    });

    axisTopic.publish(axisData);
    
    var buttonData = new ROSLIB.Message({
        data: gamepad_button
    });

    buttonTopic.publish(buttonData);
}
