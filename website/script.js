var gamepad_axis_prev = "null";
var gamepad_button_prev = "null";

// Connect to ROSBridge
var ros = new ROSLIB.Ros({
    url: 'ws://131.230.197.82:9090' // Replace with your ROSBridge server IP
});

ros.on('connection', function() {
    console.log('Connected to ROSBridge!');
});

ros.on('error', function(error) {
    console.log('Error connecting to ROSBridge:', error);
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

var diagnosticTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/diagnostics',
    messageType: 'diagnostic_msgs/DiagnosticArray'
});

// Subscribe to the topic
document.getElementById('subscribe-button').onclick = function() {
    axisTopic.subscribe(function(message) {
        document.getElementById('axis-display').innerHTML = message.data;
    });
    buttonTopic.subscribe(function(message) {
        document.getElementById('button-display').innerHTML = message.data;
    });
    diagnosticTopic.subscribe(function(message) {

        document.getElementById('diagnostic-display').innerHTML = message.status.map(status => {
            status.name + " " + status.message;
        });
    });
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
    gamepad_button = gamepad.buttons.map(button => button.value ? 1 : 0);

    // Only publish if there is a change in data
    if (gamepad_axis_prev.toString() != gamepad_axis.toString()) {
        gamepad_axis_prev = gamepad_axis;

        // Axis data ros message
        var axisData = new ROSLIB.Message({
            data: gamepad_axis
        });

        axisTopic.publish(axisData);
    }
    if (gamepad_button_prev.toString() != gamepad_button.toString()) {
        gamepad_button_prev = gamepad_button
        
        // Display button states
        var buttonData = new ROSLIB.Message({
            data: gamepad_button
        });
    
        // console.log(gamepad.axes);
        // console.log(gamepad.buttons.map(button => button.value));
        
        buttonTopic.publish(buttonData);
    }

    
}
