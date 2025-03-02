const map_canvas = document.getElementById("mapCanvas");
const lidar_canvas = document.getElementById("lidarCanvas")
const pose_canvas = document.getElementById("poseCanvas")
const map_ctx = map_canvas.getContext("2d");
const lidar_ctx = lidar_canvas.getContext("2d");
const pose_ctx = pose_canvas.getContext("2d");

const socket = io("http://192.168.12.18:5000"); // Connect to WebSocket server

const SCALE = 0.05; // Each pixel = 5cm → 0.05m
let points = []; // Store clicked points
estimated_position = [0, 0];
estimated_position_m = [];
estimated_direction_m = [];

// Load the map image
const mapImage = new Image();
mapImage.src = "map.png"; // Ensure this file is in the same directory
mapImage.onload = () => {
  map_canvas.width = mapImage.width;
  map_canvas.height = mapImage.height;
  
  lidar_canvas.width = mapImage.width;
  lidar_canvas.height = mapImage.height;

  pose_canvas.width = mapImage.width;
  pose_canvas.height = mapImage.height;

  map_ctx.drawImage(mapImage, 0, 0);
};


function onPlanPathClick(){
  socket.emit("plan_click")
}
function onInitiateClick() {
  socket.emit("init_click")
}
function onClearCostmapClick() {
  socket.emmit("clear_costmap")
}


function drawLidar(lidar_points) {
  lidar_ctx.clearRect(0, 0, lidar_canvas.width, lidar_canvas.height);
  lidar_ctx.fillStyle = "red";

  for (let i = 0; i < lidar_points.length; i++){
    // console.log("plotting point", lidar_points[i][0], lidar_points[i][1])
    lidar_ctx.fillRect(lidar_points[i][0], lidar_points[i][1], 5, 5);
  }
}


// Draw a marker based on type
function drawMarker(x, y, type) {
    map_ctx.fillStyle = 
        type === "waypoint" ? "green" : 
        type === "collect" ? "blue" : 
        type === "dropoff" ? "pink" : 
        // type === "end" ? "red" : "black"; // Default color if type is unknown
    map_ctx.beginPath();
    map_ctx.arc(x, y, 5, 0, Math.PI * 2);
    map_ctx.fill();
}
function drawRobotPose(x, y) {
  map_ctx.fillStyle = "orange";
  pixelx = x/SCALE
  pixely = y/SCALE 
  map_ctx.fillRect(pixelx,pixely,3,3);
}

// Draw a path connecting points
function drawPath() {
    if (points.length < 2) return;
    map_ctx.strokeStyle = "black";
    map_ctx.lineWidth = 1;
    map_ctx.beginPath();
    map_ctx.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i++) {
        map_ctx.lineTo(points[i].x, points[i].y);
    }
    // ctx.stroke();
}

function drawPlan(plan, color) {
  console.log("plotting plan: ", plan)
  if (plan.length < 2) return;
  map_ctx.strokeStyle = color;
  map_ctx.beginPath();
  map_ctx.moveTo(plan[0][0]/SCALE, plan[0][1]/SCALE);
  for (let i = 1; i < plan.length; i++) {
    map_ctx.lineTo(plan[i][0]/SCALE, plan[i][1]/SCALE);
  }
  map_ctx.stroke();
}


function drawEstimatePosition(x, y) {
  console.log("plotting estimated_position", [x, y]);
  pose_ctx.clearRect(0, 0, map_canvas.width, map_canvas.height);  
  pose_ctx.fillStyle = "orange";
  pose_ctx.beginPath();
  pose_ctx.arc(x, y, 5, 0, Math.PI * 2);
  pose_ctx.fill(); 
  estimated_position = [x, y];
}

function drawEstimateOrientation(x, y) {
  
  dx = estimated_position[0] - x;
  dy = estimated_position[1] - y;

  norm = Math.sqrt(dx**2 + dy**2);

  nx = dx / norm;
  ny = dy / norm; 
 
  length = 20;

  vx = estimated_position[0] - nx * length;
  vy = estimated_position[1] - ny * length;

  console.log("plotting estimate orientation" [x, y]);
  drawEstimatePosition(estimated_position[0], estimated_position[1]);
  pose_ctx.strokeStyle = "orange";
  pose_ctx.lineWidth = 3;
  pose_ctx.beginPath();
  pose_ctx.moveTo(estimated_position[0], estimated_position[1]);
  pose_ctx.lineTo(vx, vy);
  pose_ctx.stroke();

}


// Capture click coordinates
pose_canvas.addEventListener("click", (event) => {
    const rect = map_canvas.getBoundingClientRect();
    const xPixel = event.clientX - rect.left;
    const yPixel = event.clientY - rect.top;
    const yPixelM = rect.bottom - event.clientY;

    // Convert pixels to meters
    const xMeters = (xPixel * SCALE).toFixed(2);
    const yMeters = (yPixelM * SCALE).toFixed(2);

    // Get selected point type
    const pointType = document.getElementById("pointType").value;
    
    if (pointType == "estimate_position"){
      estimated_position = [xPixel, yPixel];
      estimated_position_m = [xMeters, yMeters];
      drawEstimatePosition(xPixel, yPixel);
      return;
    }
    else if (pointType == "estimate_direction"){
      drawEstimateOrientation(xPixel, yPixel);
      estimated_direction_m = [xMeters, yMeters];
      socket.emit("pose_estimated", {position:estimated_position_m, direction:estimated_direction_m});
      return;
    }

    // Store point with type
    points.push({ x: xPixel, y: yPixel, xm: xMeters, ym: yMeters, type: pointType });

    // Redraw map
    map_ctx.drawImage(mapImage, 0, 0);
    points.forEach((p) => drawMarker(p.x, p.y, p.type));
    drawPath();

    // Update UI
    document.getElementById("coordinates").innerText = `Last: ${pointType} - X=${xMeters}m, Y=${yMeters}m`;

    // Send data to Python
    socket.emit("map_click", { points: points });    
});


socket.on("path_update", (data) => {
  console.log("Recived path: ", data.plan_array);
  document.getElementById("path_status").innerText = data.status
  if (data.type == "full"){
    drawPlan(data.plan_array, "green")
  }
  else {
    drawPlan(data.plan_array, "red")
  }

})

socket.on("position_update", (data) => {
  document.getElementById("position_x").innerText = data.position.x;
  document.getElementById("position_y").innerText = data.position.y;
  drawRobotPose(data.position.x, data.position.y);
})
socket.on("lidar_update", (data) => {
  // console.log("lidar recived", data.status);
  // for (let i = 1; i < data.points.length; i++){
  //   console.log("point ", i, data.points[i]);
  // }
  drawLidar(data.points)
})

socket.on("grasping_path_status", (data) => {
  document.getElementById("grasping_path_trigger").innerText = data.trigger;
  document.getElementById("grasping_path_index").innerText = data.path_index; 
})

socket.on("item_speed", (data) => {
  document.getElementById("item_speed_dx").innerText = data.dx;
  document.getElementById("item_speed_dy").innerText = data.dy;
})
