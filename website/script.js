

// Connect to ROSBridge
var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090' // Replace with your ROSBridge server IP
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
    messageType: 'std_msgs/Int32MultiArray' // Replace with the topic's message type
});

// Create a ROS topic
var buttonTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/gamepad_button', // Replace with your desired topic
    messageType: 'std_msgs/Int32MultiArray' // Replace with the topic's message type
});

// Subscribe to the topic
document.getElementById('subscribe-button').onclick = function() {
    axisTopic.subscribe(function(message) {
        document.getElementById('data-display').innerHTML = message.data;
    });
};

// Controller polling using the Gamepad API
window.addEventListener("gamepadconnected", function(e) {
    console.log("Gamepad connected!");
    setInterval(readControllerData, 50); // Poll for updates
});

function readControllerData() {
    var gamepad = navigator.getGamepads()[0]; // Assuming the first connected gamepad
    
    if (gamepad == undefined) {
	return;
	}

    // Display axis data
    var axisData = new ROSLIB.Message({
    	data: gamepad.axes.map(axis => parseInt(axis.toFixed(2)*100))
    });
    
    // Display button states
    var buttonData = new ROSLIB.Message({
    	data: gamepad.buttons.map(button => parseInt(button.pressed ? '1' : '0'))
    });
    
    axisTopic.publish(axisData);
    buttonTopic.publish(buttonData);
    
}
