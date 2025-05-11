// Parameters - change these to match the Pi
var ip = "131.230.197.84"; // Default IP of the Pi for local development
var port = "9090"; // Port of webserver node

// Automatically use the current hostname if we're not on localhost or using file protocol
if (
  window.location.protocol !== "file:" &&
  window.location.hostname !== "localhost" &&
  window.location.hostname !== "127.0.0.1"
) {
  ip = window.location.hostname;
  console.log("Using hostname for ROS connection:", ip);
}

var url_string = `ws://${ip}:${port}`;
var gamepad_axis_prev = "null";
var gamepad_button_prev = "null";
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

// Camera node topics
var cameraNodeImageTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/camera_node/usb_camera_image", // Changed to match the new topic name in launch file
  messageType: "std_msgs/String",
});
var cameraNodeRS1ColorTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/camera_node/rs1_color_image",
  messageType: "std_msgs/String",
});
var cameraNodeRS1DepthTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/camera_node/rs1_depth_image",
  messageType: "std_msgs/String",
});
var cameraNodeRS2ColorTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/camera_node/rs2_color_image",
  messageType: "std_msgs/String",
});
var cameraNodeRS2DepthTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/camera_node/rs2_depth_image",
  messageType: "std_msgs/String",
});

// Diagnostic topics
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
var cameraNodeDiagTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/camera_node/diag",
  messageType: "diagnostic_msgs/DiagnosticStatus",
});

// Topic for I2C command history from controller_parser only
var controllerI2CHistoryTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/controller_parser/i2c_history",
  messageType: "std_msgs/String",
});

// Subscribe to camera node topics
cameraNodeImageTopic.subscribe(function (message) {
  updateImageWithMimeType(
    document.getElementById("camera_node_video"),
    message.data
  );
});

cameraNodeRS1ColorTopic.subscribe(function (message) {
  updateImageWithMimeType(
    document.getElementById("camera_node_rs1_color"),
    message.data
  );
});

cameraNodeRS1DepthTopic.subscribe(function (message) {
  updateImageWithMimeType(
    document.getElementById("camera_node_rs1_depth"),
    message.data
  );
});

cameraNodeRS2ColorTopic.subscribe(function (message) {
  updateImageWithMimeType(
    document.getElementById("camera_node_rs2_color"),
    message.data
  );
});

cameraNodeRS2DepthTopic.subscribe(function (message) {
  updateImageWithMimeType(
    document.getElementById("camera_node_rs2_depth"),
    message.data
  );
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

cameraNodeDiagTopic.subscribe(function (message) {
  document.getElementById("camera_node_diag").innerHTML = message.message;

  // Extract and display latency information from diagnostic values
  let latencyInfo = {};
  let hasLatencyData = false;

  if (message.values && message.values.length > 0) {
    message.values.forEach(function (kv) {
      // Check for latency values in the format: camera_key_latency_type
      if (kv.key.includes("_latency_ms")) {
        hasLatencyData = true;
        const parts = kv.key.split("_");
        const cameraType = parts[0]; // e.g., 'main_camera', 'rs1_color'
        const metricType = parts[parts.length - 2]; // e.g., 'last', 'avg', 'max'

        // Initialize camera entry if it doesn't exist
        if (!latencyInfo[cameraType]) {
          latencyInfo[cameraType] = {};
        }

        // Store the latency value
        latencyInfo[cameraType][metricType] = parseFloat(kv.value).toFixed(2);
      }

      // Also capture frame counts
      if (kv.key.includes("_frames")) {
        const cameraType = kv.key.split("_")[0];
        if (!latencyInfo[cameraType]) {
          latencyInfo[cameraType] = {};
        }
        latencyInfo[cameraType]["frames"] = kv.value;
      }
    });
  }

  // Display the latency information
  const latencyPanel = document.getElementById("camera_latency_panel");

  if (hasLatencyData) {
    // Get camera names for consistent ordering
    const cameraNames = Object.keys(latencyInfo).sort();

    let tableHTML = `
      <table class="table table-sm table-striped compact-table mb-0">
        <thead>
          <tr>
            <th>Camera</th>
            <th>Last (ms)</th>
            <th>Avg (ms)</th>
            <th>Max (ms)</th>
            <th>Frames</th>
          </tr>
        </thead>
        <tbody>
    `;

    cameraNames.forEach(function (camera) {
      const info = latencyInfo[camera];
      tableHTML += `
        <tr>
          <td>${formatCameraName(camera)}</td>
          <td>${colorCodeLatency(info.last)}</td>
          <td>${colorCodeLatency(info.avg)}</td>
          <td>${colorCodeLatency(info.max)}</td>
          <td>${info.frames || "0"}</td>
        </tr>
      `;
    });

    tableHTML += `
        </tbody>
      </table>
    `;

    latencyPanel.innerHTML = tableHTML;
  } else {
    latencyPanel.innerHTML = `
      <div class="text-center text-muted py-2">
        <small>No latency data available</small>
      </div>
    `;
  }
});

// Helper function to format camera names for display
function formatCameraName(cameraKey) {
  const nameMap = {
    main_camera: "Main Camera",
    rs1_color: "RS1 RGB",
    rs1_depth: "RS1 Depth",
    rs2_color: "RS2 RGB",
    rs2_depth: "RS2 Depth",
  };

  return nameMap[cameraKey] || cameraKey;
}

// Helper function to apply color coding to latency values based on thresholds
function colorCodeLatency(value) {
  if (!value || value === "N/A") return `<span>${value}</span>`;

  const latencyValue = parseFloat(value);
  let className = "text-success";

  if (latencyValue > 50) {
    className = "text-danger";
  } else if (latencyValue > 20) {
    className = "text-warning";
  }

  return `<span class="latency-value ${className}">${value}</span>`;
}

// Subscribe to I2C command history topic from controller_parser only
controllerI2CHistoryTopic.subscribe(function (message) {
  updateI2CCommandHistory(message.data);
});

// Function to update I2C command history display
function updateI2CCommandHistory(dataStr) {
  try {
    const data = JSON.parse(dataStr);
    const historyPanel = document.getElementById("i2c_command_history");

    // Clear the panel if it contains the waiting message
    if (historyPanel.querySelector(".text-muted")) {
      historyPanel.innerHTML = "";
    }

    // Sort devices by address to maintain consistent order
    const addresses = Object.keys(data).sort(
      (a, b) => parseInt(a) - parseInt(b)
    );

    // Generate HTML for each device
    let htmlContent = "";

    addresses.forEach((address) => {
      const device = data[address];
      const deviceName = device.device_name || "Unknown Device";
      const lastCommand = device.last_command || "";
      const timestamp = device.timestamp || "";

      htmlContent += `
            <div class="i2c-item">
                <div class="d-flex justify-content-between">
                    <strong>${deviceName}</strong>
                    <small class="text-muted">${timestamp}</small>
                </div>
                <div>
                    <span class="badge bg-secondary">${device.address}</span>
                    <span class="ms-2">${lastCommand}</span>
                </div>
            </div>`;
    });

    // Update the panel content if we have new data
    if (htmlContent) {
      historyPanel.innerHTML = htmlContent;
    }
  } catch (e) {
    console.error("Error parsing I2C history data:", e);
  }
}

// Connect gamepad
window.addEventListener("gamepadconnected", function (e) {
  console.log("Gamepad connected!");
  controllerConnected = true;

  // Update UI to show controller connected
  document.getElementById("controller-status").classList.remove("disconnected");
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

// Function to update controller value displays in the mapping table
function updateControllerValues(gamepad) {
  // Update button values
  if (gamepad.buttons) {
    // Standard button mapping
    updateControlValueElement(
      "button-a",
      gamepad.buttons[0].value > 0
        ? Math.round(gamepad.buttons[0].value * 100)
        : 0
    );
    updateControlValueElement(
      "button-b",
      gamepad.buttons[1].value > 0
        ? Math.round(gamepad.buttons[1].value * 100)
        : 0
    );
    updateControlValueElement(
      "button-x",
      gamepad.buttons[2].value > 0
        ? Math.round(gamepad.buttons[2].value * 100)
        : 0
    );
    updateControlValueElement(
      "button-y",
      gamepad.buttons[3].value > 0
        ? Math.round(gamepad.buttons[3].value * 100)
        : 0
    );
    updateControlValueElement(
      "button-lb",
      gamepad.buttons[4].value > 0
        ? Math.round(gamepad.buttons[4].value * 100)
        : 0
    );
    updateControlValueElement(
      "button-rb",
      gamepad.buttons[5].value > 0
        ? Math.round(gamepad.buttons[5].value * 100)
        : 0
    );
    updateControlValueElement(
      "button-lt",
      gamepad.buttons[6].value > 0
        ? Math.round(gamepad.buttons[6].value * 100)
        : 0
    );
    updateControlValueElement(
      "button-rt",
      gamepad.buttons[7].value > 0
        ? Math.round(gamepad.buttons[7].value * 100)
        : 0
    );
    updateControlValueElement(
      "button-back",
      gamepad.buttons[8].value > 0
        ? Math.round(gamepad.buttons[8].value * 100)
        : 0
    );
    updateControlValueElement(
      "button-start",
      gamepad.buttons[9].value > 0
        ? Math.round(gamepad.buttons[9].value * 100)
        : 0
    );
    updateControlValueElement(
      "button-ls",
      gamepad.buttons[10].value > 0
        ? Math.round(gamepad.buttons[10].value * 100)
        : 0
    );
    updateControlValueElement(
      "button-rs",
      gamepad.buttons[11].value > 0
        ? Math.round(gamepad.buttons[11].value * 100)
        : 0
    );
  }

  // Update axis values
  if (gamepad.axes) {
    updateControlValueElement(
      "left-stick-x",
      Math.round(gamepad.axes[0] * 100)
    );
    updateControlValueElement(
      "left-stick-y",
      Math.round(gamepad.axes[1] * 100)
    );
    updateControlValueElement(
      "right-stick-x",
      Math.round(gamepad.axes[2] * 100)
    );
    updateControlValueElement(
      "right-stick-y",
      Math.round(gamepad.axes[3] * 100)
    );

    // DPad values
    if (Math.abs(gamepad.axes[6]) > 0) {
      if (gamepad.axes[6] < 0) {
        updateControlValueElement(
          "dpad-left",
          Math.abs(Math.round(gamepad.axes[6] * 100))
        );
        updateControlValueElement("dpad-right", 0);
      } else {
        updateControlValueElement(
          "dpad-right",
          Math.round(gamepad.axes[6] * 100)
        );
        updateControlValueElement("dpad-left", 0);
      }
    } else {
      updateControlValueElement("dpad-left", 0);
      updateControlValueElement("dpad-right", 0);
    }

    if (Math.abs(gamepad.axes[7]) > 0) {
      if (gamepad.axes[7] < 0) {
        updateControlValueElement(
          "dpad-up",
          Math.abs(Math.round(gamepad.axes[7] * 100))
        );
        updateControlValueElement("dpad-down", 0);
      } else {
        updateControlValueElement(
          "dpad-down",
          Math.round(gamepad.axes[7] * 100)
        );
        updateControlValueElement("dpad-up", 0);
      }
    } else {
      updateControlValueElement("dpad-up", 0);
      updateControlValueElement("dpad-down", 0);
    }
  }
}

// Helper function to update an individual control value in the mapping table
function updateControlValueElement(controlId, value) {
  const valueElement = document.querySelector(
    `.controller-value[data-control="${controlId}"]`
  );
  if (valueElement) {
    const absValue = Math.abs(value);
    if (absValue === 0) {
      valueElement.textContent = "-";
      valueElement.classList.remove("text-primary", "fw-bold");
    } else {
      valueElement.textContent = value;
      valueElement.classList.add("text-primary", "fw-bold");
    }
  }
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
    activeControllerElements.add(gamepad_axis[7] < 0 ? "dpad-up" : "dpad-down");

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

  // Update value displays in the mapping table
  updateControllerValues(gamepad);

  var axisData = new ROSLIB.Message({
    data: gamepad_axis,
  });

  axisTopic.publish(axisData);

  var buttonData = new ROSLIB.Message({
    data: gamepad_button,
  });

  buttonTopic.publish(buttonData);
}
