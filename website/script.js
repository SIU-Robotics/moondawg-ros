// Parameters - change these to match the Pi
var ip = "131.230.197.84"; // IP of the Pi
var port = "9090"; // Port of webserver node

var gamepad_axis_prev = "null";
var gamepad_button_prev = "null";
var url_string = `ws://${ip}:${port}`;
var controllerConnected = false;
var activeControllerElements = new Set(); // Track active controller elements

// Connect to and set up ros bridge
var ros = new ROSLIB.Ros({
    url: url_string,
});

ros.on("connection", function () {
    console.log("Connected to ROSBridge!");

    var connectionStatus = new ROSLIB.Message({
        data: 0,
    });

    // Update UI to show connected
    document.getElementById("ros-status").classList.remove("disconnected");
    document.getElementById("ros-status").classList.add("connected");
    document.getElementById("ros-status-text").textContent = "Connected";

    setInterval(() => connectionTopic.publish(connectionStatus), 250);

    // Fetch parameters when connected
    fetchParameters();
});

ros.on("error", function (error) {
    console.log("Error connecting to ROSBridge:", error);
    document.getElementById("ros-status").classList.remove("connected");
    document.getElementById("ros-status").classList.add("disconnected");
    document.getElementById("ros-status-text").textContent = "Disconnected";
});

ros.on("close", function () {
    document.getElementById("ros-status").classList.remove("connected");
    document.getElementById("ros-status").classList.add("disconnected");
    document.getElementById("ros-status-text").textContent = "Disconnected";
    setTimeout(() => ros.connect(url_string), 5000); // Retry connection every 5000 ms
});

// Define topics
var connectionTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/connection_status",
    messageType: "std_msgs/Byte",
});
var axisTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/controller_parser/gamepad_axis",
    messageType: "std_msgs/Int8MultiArray",
});
var buttonTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/controller_parser/gamepad_button",
    messageType: "std_msgs/Int8MultiArray",
});
var imageTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/controller_parser/compressed_image",
    messageType: "std_msgs/String",
});
var controllerDiagTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/controller_parser/diag",
    messageType: "diagnostic_msgs/DiagnosticStatus",
});
var i2cDiagTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/i2c_node/diag",
    messageType: "diagnostic_msgs/DiagnosticStatus",
});

// Parameters
// Replace the incorrect ros.getParams function with proper ROSLIB parameter fetching
function fetchParameters() {
    // Get a list of all parameters
    var getParamNamesClient = new ROSLIB.Service({
        ros: ros,
        name: "/controller_parser/get_parameters/belt_speed_index",
        serviceType: "rosapi/GetParamNames",
    });

    getParamNamesClient.callService(
        new ROSLIB.ServiceRequest({}),
        function (result) {
            console.log(result);
            const paramNames = result;
            let params = {};
            let fetchedCount = 0;

            // For each parameter name, get its value
            paramNames.forEach(function (paramName) {
                var param = new ROSLIB.Param({
                    ros: ros,
                    name: paramName,
                });

                param.get(function (value) {
                    params[paramName] = value;
                    fetchedCount++;

                    // When all parameters are fetched, display them
                    if (fetchedCount === paramNames.length) {
                        const formattedParams = JSON.stringify(params, null, 2);
                        document.getElementById(
                            "params-display"
                        ).innerHTML = `<pre>${formattedParams}</pre>`;
                    }
                });
            });
        }
    );
}

// Subscribe to topics
imageTopic.subscribe(function (message) {
    document.getElementById("video_out").src =
        "data:image/jpeg;base64," + message.data;
});

axisTopic.subscribe(function (message) {
    // Format axis data in a more readable way
    if (Array.isArray(message.data)) {
        const axisNames = [
            "Left X",
            "Left Y",
            "Right X",
            "Right Y",
            "LT",
            "RT",
            "DPad X",
            "DPad Y",
        ];
        let formattedData = "";

        for (let i = 0; i < message.data.length; i++) {
            const axisName = i < axisNames.length ? axisNames[i] : `Axis ${i}`;
            formattedData += `${axisName}: ${message.data[i]}\n`;
        }

        document.getElementById("axis-display").innerHTML = formattedData;
    } else {
        document.getElementById("axis-display").innerHTML = message.data;
    }
});

buttonTopic.subscribe(function (message) {
    // Format button data in a more readable way
    if (Array.isArray(message.data)) {
        const buttonNames = [
            "A",
            "B",
            "X",
            "Y",
            "LB",
            "RB",
            "Back",
            "Start",
            "LS",
            "RS",
        ];
        let formattedData = "";

        for (let i = 0; i < message.data.length; i++) {
            const buttonName =
                i < buttonNames.length ? buttonNames[i] : `Button ${i}`;
            const value = message.data[i];
            formattedData += `${buttonName}: ${value}\n`;
        }

        document.getElementById("button-display").innerHTML = formattedData;
    } else {
        document.getElementById("button-display").innerHTML = message.data;
    }
});

controllerDiagTopic.subscribe(function (message) {
    document.getElementById("controller_diag").innerHTML = message.message;
});

i2cDiagTopic.subscribe(function (message) {
    document.getElementById("i2c_diag").innerHTML = message.message;
});

// Connect gamepad
window.addEventListener("gamepadconnected", function (e) {
    console.log("Gamepad connected!");
    controllerConnected = true;

    // Update UI to show controller connected
    document
        .getElementById("controller-status")
        .classList.remove("disconnected");
    document.getElementById("controller-status").classList.add("connected");
    document.getElementById("controller-status-text").textContent = "Connected";

    setInterval(readControllerData, 75); // Read from controller every 75 ms
});

// Listen for gamepad disconnection
window.addEventListener("gamepaddisconnected", function (e) {
    console.log("Gamepad disconnected!");
    controllerConnected = false;

    // Update UI to show controller disconnected
    document.getElementById("controller-status").classList.remove("connected");
    document.getElementById("controller-status").classList.add("disconnected");
    document.getElementById("controller-status-text").textContent =
        "Disconnected";
});

// Function to update the controller mapping highlight
function updateControllerHighlights(elements) {
    // Reset all highlights
    document.querySelectorAll(".controller-mapping-row").forEach((row) => {
        row.classList.remove("table-primary");
    });

    // Apply highlights to active elements
    elements.forEach((element) => {
        const row = document.getElementById(`mapping-${element}`);
        if (row) {
            row.classList.add("table-primary");
        }
    });
}

// Function to be called every time the controlled is read from
function readControllerData() {
    var gamepad = navigator.getGamepads()[0]; // Assuming the first connected gamepad

    if (gamepad == undefined) {
        return;
    }

    if (!ros.isConnected) {
        return;
    }

    gamepad_axis = gamepad.axes.map((axis) => parseInt(axis.toFixed(2) * 100));
    gamepad_button = gamepad.buttons.map(
        (button) => (button.value = parseInt(button.value.toFixed(2) * 100))
    );

    // Track active controller elements
    activeControllerElements.clear();

    // Check axes
    if (Math.abs(gamepad_axis[0]) > 10)
        activeControllerElements.add("left-stick-x");
    if (Math.abs(gamepad_axis[1]) > 10)
        activeControllerElements.add("left-stick-y");
    if (Math.abs(gamepad_axis[2]) > 10)
        activeControllerElements.add("right-stick-x");
    if (Math.abs(gamepad_axis[3]) > 10)
        activeControllerElements.add("right-stick-y");
    if (Math.abs(gamepad_axis[6]) > 0)
        activeControllerElements.add(
            gamepad_axis[6] < 0 ? "dpad-left" : "dpad-right"
        );
    if (Math.abs(gamepad_axis[7]) > 0)
        activeControllerElements.add(
            gamepad_axis[7] < 0 ? "dpad-up" : "dpad-down"
        );

    // Check buttons (standard mapping)
    if (gamepad_button[0] > 0) activeControllerElements.add("button-a");
    if (gamepad_button[1] > 0) activeControllerElements.add("button-b");
    if (gamepad_button[2] > 0) activeControllerElements.add("button-x");
    if (gamepad_button[3] > 0) activeControllerElements.add("button-y");
    if (gamepad_button[4] > 0) activeControllerElements.add("button-lb");
    if (gamepad_button[5] > 0) activeControllerElements.add("button-rb");
    if (gamepad_button[6] > 0) activeControllerElements.add("button-lt");
    if (gamepad_button[7] > 0) activeControllerElements.add("button-rt");
    if (gamepad_button[8] > 0) activeControllerElements.add("button-back");
    if (gamepad_button[9] > 0) activeControllerElements.add("button-start");
    if (gamepad_button[10] > 0) activeControllerElements.add("button-ls");
    if (gamepad_button[11] > 0) activeControllerElements.add("button-rs");

    // Update highlights in the UI
    updateControllerHighlights(activeControllerElements);

    var axisData = new ROSLIB.Message({
        data: gamepad_axis,
    });

    axisTopic.publish(axisData);

    var buttonData = new ROSLIB.Message({
        data: gamepad_button,
    });

    buttonTopic.publish(buttonData);
}
