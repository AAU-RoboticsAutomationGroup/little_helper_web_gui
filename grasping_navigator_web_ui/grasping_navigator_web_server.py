from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
import rclpy.time
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import tf2_ros
from flask import Flask, send_file, send_from_directory
from flask_socketio import SocketIO
from threading import Thread
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid  
from sensor_msgs.msg import LaserScan 
from lh_interfaces.msg import PathStatus
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import TwistStamped 

import numpy as np
import yaml
import ast
# ROS2 Node Initialization
import os
import shutil
import cv2   


global planned_path
planned_path = []

global webui_waypoints
webui_waypoints = []


# global map_path
# map_path = "/home/a/work/test_map1.png"
#
# path_to_map_yml = "/home/a/work/test_map_1.yaml"
# with open(path_to_map_yml) as stream:
#     try:
#         map_data = yaml.safe_load(stream)
#     except yaml.YAMLError as exc:
#         print(exc)

from ament_index_python.packages import get_package_share_directory
self_share_dir = get_package_share_directory('grasping_navigator_web_ui')

# path_to_image = path_to_map_yml.replace("yaml","png")



app = Flask(__name__, static_folder=os.path.join(self_share_dir, "data"), static_url_path="")
socketio = SocketIO(app, cors_allowed_origins="*")


class Webui(Node):
    def __init__(self):
        super().__init__('navigate_grasping_path_webui')

        self.declare_parameter("map_path", rclpy.Parameter.Type.STRING)

        map_path = self.get_parameter("map_path").value
        with open(map_path) as map_description:
            map_data = yaml.safe_load(map_description)
        self.get_logger().info(f"map data: {map_data}") 
        map_image_name = map_data['image']

        map_dir = os.path.dirname(map_path)
        if map_image_name.split(".")[-1] == "pgm":
            map_tmp = cv2.imread(os.path.join(map_dir, map_image_name))
            cv2.imwrite(os.path.join(self_share_dir, "data", "map.png"), map_tmp)
            self.get_logger().info(f"saved map converted from pgm at {os.path.join(self_share_dir, 'data', 'map.png')} ")
        elif map_image_name.split(".")[-1] == "pgm": 
            shutil.copyfile(os.path.join(map_dir, map_image_name), os.path.join(self_share_dir,"data","map.png"))
            self.get_logger().info(f"saved map at {os.path.join(self_share_dir, 'data', 'map.png')} ")
        else:
            raise Exception(f"The map file type ({map_image_name.split('.')[-1]}) is not supported please use .pgm or .png")
    

        self.tf_buffer = tf2_ros.Buffer() 

        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.parent_frame_id = "map"
        self.robot_base_frame_id = "base_link"
        self.create_timer(0.1, self.get_robot_position_timer_callback)

        self.path_publisher = self.create_publisher(String, 'webui_waypoints', 10)  # Topic for clicks
        self.initialize_nav_publisher = self.create_publisher(Bool, 'webui_move_trigger',10)
        self.get_logger().info("Flask/ROS2 Node Initialized")
        
        self.create_subscription(Path, "plan", callback=self.recived_path_callback, qos_profile= 10) 
        self.create_subscription(Path, "grasping_path", callback=self.grasping_path_plan_callback, qos_profile = 10)

        self.create_subscription(OccupancyGrid, "global_costmap/costmap", self.costmap_callback, 10)
        self.cmap_origin = [0, 0]
        self.client = self.create_client(Empty, '/global_costmap/clear_entirely_global_costmap')
        self.cmap_height = 1
        self.cmap_width = 1
        self.cmap_resolution = 1 
        

        self.lidar_points = []
        self.create_subscription(LaserScan, "scan", self.lidar_callback, 10)

        self.estimated_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)

        #navigation feedback 
        self.create_subscription(PathStatus, "grasping_path_status", self.grasping_path_status_callback, 10)
      
        self.arm_base_vel_subscriber = self.create_subscription(TwistStamped, '/vel_arm_base', self.send_arm_vel_callback, 10)

        # clear costmap client
        self.clear_costmap_client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')

    def clear_costmap(self):
        req = ClearEntireCostmap()
        future = self.clear_costmap_client.call_async(req)
        future.add_done_callback(self.clear_costmap_response_callback)

    def clear_costmap_response_callback(self, future):
        self.get_logger().info("costmap cleared")


    def send_arm_vel_callback(self, msg):
        dx = msg.twist.linear[0]
        dy = msg.twist.linear[1]
        socketio.emit("item_speed",{"dx":dx,"dy":dy})

    def grasping_path_status_callback(self, msg):
        trigger = msg.trigger
        path_index = msg.current_path_index
        socketio.emit("grasping_path_status",{"trigger":trigger,"path_index":path_index})

    def grasping_path_plan_callback(self, msg):
        self.get_logger().info("recived grasping path from planner")
        self.get_logger().debug(f"{msg.poses}")
        planned_path = self.ros_path_to_json(msg) 
        self.get_logger().info(f"planned path {planned_path}")
        socketio.emit("path_update", {"status":"plan recieved", "plan_array":planned_path, "type":"grasping"})

    def get_robot_position_timer_callback(self):
        try:
            robot_position = self.tf_buffer.lookup_transform(self.parent_frame_id, self.robot_base_frame_id, rclpy.time.Time())

            relative_x = robot_position.transform.translation.x - self.cmap_origin[0]
            relative_y = ((self.cmap_height*self.cmap_resolution) - robot_position.transform.translation.y) + self.cmap_origin[1]

            socketio.emit("position_update", 
                          {"position":{"x":round(relative_x, 3), 
                                       "y":round(relative_y, 3)}})
            # self.get_logger().info(f"send lidar update {self.lidar_points}")
            
            socketio.emit("lidar_update", {"status": "lidar", "points": self.lidar_points})
        except Exception as e:
            self.get_logger().warn(f"{e}")

    def recived_path_callback(self, msg):
        self.get_logger().info("recived plan from planner")
        self.get_logger().debug(f"{msg.poses}")
        planned_path = self.ros_path_to_json(msg) 
        self.get_logger().info(f"planned path {planned_path}")
        socketio.emit("path_update", {"status":"plan recieved", "plan_array":planned_path, "type":"full"})

    def costmap_callback(self, msg):
        """
        when a new costmap is recived, it is converted to a numpy array for waypoint generation 
        """
        # print(msg.info)
        self.costmap = (np.array(msg.data).reshape((msg.info.height, msg.info.width,1)))
        self.cmap_height = msg.info.height
        self.cmap_width = msg.info.width
        self.cmap_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.get_logger().info(f"costmap origin {self.cmap_origin}")
        self.cmap_resolution = msg.info.resolution
    

    def ros_path_to_json(self, ros_path : Path):
        coordinates = []
        self.get_logger().info("converting ros plan to xy list")
        self.get_logger().info(f"{len(ros_path.poses)}")
        for pose in ros_path.poses:
            self.get_logger().debug(f"pose: {pose}")
            # the y axis is flipped on the webui so funny buisness is here 
            coordinates.append([float(pose.pose.position.x) - self.cmap_origin[0], 
                                ((self.cmap_height*self.cmap_resolution) - float(pose.pose.position.y) ) + self.cmap_origin[1]
                                ])
        
        # print(f"plan coordinates: {coordinates}")
        return coordinates


    def publish_points(self, path):
        msg = String()
        path_buffer = [] 
        path_buffer += path

        if len(path) == 0 or len(self.cmap_origin) == 0:
            self.get_logger().info(f"no points {len(path)} or costmap origin {len(self.cmap_origin)}")
            return 0  

        for point in path_buffer:
            point['xm'] = float(point["xm"]) + self.cmap_origin[0]
            point['ym'] = float(point["ym"]) + self.cmap_origin[1]

        msg.data = str(path_buffer)
        self.get_logger().info(f"Published path: {path_buffer}")
        self.path_publisher.publish(msg)

    def publish_init(self):
        msg = Bool()
        msg.data = True
        self.get_logger().info("sending navigation init signal")
        self.initialize_nav_publisher.publish(msg)

    def publish_estimated_pose(self, center_position, direction_position):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.parent_frame_id
        msg.pose.pose.position.x = float(center_position[0]) + self.cmap_origin[0]
        msg.pose.pose.position.y = float(center_position[1]) + self.cmap_origin[1]

        dx = float(direction_position[1]) - float(center_position[1])
        dy = float(direction_position[0]) - float(center_position[0])

        norm = np.linalg.norm(np.array([dx, dy]))
        
        self.get_logger().info(f"norm {norm}, dx {dx}, dy {dy}")

        nx = dx/norm 
        ny = dy/norm

        heading_angle = np.arctan2(nx, ny)

        qw = np.cos(heading_angle/2)
        qz = np.sin(heading_angle/2)
        
        msg.pose.pose.orientation.w = float(qw)
        msg.pose.pose.orientation.z = float(qz) 
        
        self.estimated_pose_publisher.publish(msg)
    
    def lidar_callback(self, lidar_data : LaserScan):
        # self.get_logger().info("lidar recived")
        frame_id = lidar_data.header.frame_id
        ranges = np.array(lidar_data.ranges)
        
        filtered_ranges = np.zeros_like(ranges)
        filtered_ranges[np.isfinite(ranges)] = ranges[np.isfinite(ranges)]
        ranges = filtered_ranges

        radians_array = np.linspace(lidar_data.angle_min, lidar_data.angle_max, ranges.shape[0])

        xyzo = np.vstack([np.cos(radians_array)*ranges, np.sin(radians_array)*ranges, np.zeros_like(ranges), np.ones_like(ranges)])

        try:
            lidar_tf = self.tf_buffer.lookup_transform(self.parent_frame_id, frame_id, rclpy.time.Time())
            # self.get_logger().info(f"transform {lidar_tf}")
            tf = self.cast_tf_matrix(lidar_tf.transform)
            # self.get_logger().info(f"transform \n{tf}")
          
            map_xyzo = (tf @ xyzo).T
            # map_xyzo = xyzo.T
            map_x_m = map_xyzo[:,0] - self.cmap_origin[0]
            map_y_m = (self.cmap_height*self.cmap_resolution - map_xyzo[:,1]) + self.cmap_origin[1]


            map_xy_m = np.vstack([map_x_m, map_y_m])
            
            map_xy_m = map_xy_m[np.sqrt(np.sum(map_xy_m**2, axis=1)) > 0.2]

            # map_xy_m[:,0] = map_xy_m[:,0] - self.cmap_origin[0] 
            # map_xy_m[:,1] = map_xy_m[:,1] - self.cmap_origin[1]

            map_xy_p = map_xy_m / self.cmap_resolution 


            self.lidar_points = (map_xy_p.T).tolist()
            
            # self.get_logger().info(f"{map_xyzo.shape}")

        except Exception as e: 
            self.get_logger().info(f"{e}")


        # self.get_logger().info(f"{ranges.shape}")
        
    
    def cast_tf_matrix(self, transform) -> np.ndarray:
        """
        cast the ros2 transform into a transformation matrix
        """
        tf_matrix = np.zeros((4,4))
        tf_matrix[3,3]=1
        tf_matrix[0,3] = transform.translation.x
        tf_matrix[1,3] = transform.translation.y
        tf_matrix[2,3] = transform.translation.z
        qx = transform.rotation.x
        qy = transform.rotation.y
        qz = transform.rotation.z
        qw = transform.rotation.w

        tf_matrix[0,0] = 1 - 2*qy**2 - 2*qz**2
        tf_matrix[0,1] = 2*qx*qy - 2*qw*qz 
        tf_matrix[0,2] = 2*qx*qz + 2*qw*qy
        tf_matrix[1,0] = 2*qx*qy + 2*qw*qz
        tf_matrix[1,1] = 1 - 2*qx**2 - 2*qz**2
        tf_matrix[1,2] = 2*qy*qz - 2*qw*qx 
        tf_matrix[2,0] = 2*qx*qz - 2*qw*qy 
        tf_matrix[2,1] = 2*qy*qz - 2*qw*qx 
        tf_matrix[2,2] = 1 - 2*qx**2 - 2*qy**2 

        return tf_matrix



def run_flask_app():
    # Start the Flask application
    # Flask App Initialization

# Create a Flask route to serve the map and interact with web
    @app.route("/")
    def serve_html():
        return send_from_directory(os.path.join(self_share_dir, 'data'), "index.html")
    @app.route("/ajax/libs/socket.io/4.5.4/socket.io.js")
    def serve_socketio_js():
        return send_from_directory(os.path.join(self_share_dir, 'data'), "socket.io.js")

    @app.route("/script.js")
    def serve_js():
        print(f"path to js {os.path.join(self_share_dir, 'data')}")
        return send_from_directory(os.path.join(self_share_dir, 'data'), "script.js")

    @app.route("/map.png")
    def serve_image():
        return send_from_directory(os.path.join(self_share_dir, 'data'), "map.png")



    @app.route("/")
    def serve_planned_path():
        return planned_path


# Handle clicks from JavaScript
    @socketio.on("map_click")
    def handle_map_click(data):
        global webui_waypoints
        path = data["points"]
        print("User path (meters) with types:")
        for point in path:
            print(f"   Type={point['type']} | X={point['xm']}m, Y={point['ym']}m")
        print("-" * 30)  # Separator in logs
        webui_waypoints = path
        # Send the path to ROS2 topic


    @socketio.on("init_click")
    def handle_init_click():
        print("init navigation")
        ros2_node.publish_init()


    @socketio.on("plan_click")
    def handle_plan_click():
        global webui_waypoints
        print("create plan")
        ros2_node.publish_points(webui_waypoints)

    @socketio.on("pose_estimated")
    def handle_estimate_pose_click(data):
        ros2_node.get_logger().info(f"got pose estimation data {data}")
        position = data['position']
        orientation = data['direction']
        ros2_node.publish_estimated_pose(position, orientation)

    @socketio.on("clear_costmap")
    def handle_clear_costmap():
        ros2_node.clear_costmap()


    socketio.run(app, debug=True, use_reloader=False, allow_unsafe_werkzeug=True, host="192.168.12.18")


def main(args=None):
    rclpy.init(args=args)
    
    # Initialize ROS2 Node
    global ros2_node
    ros2_node = Webui()

    # Start Flask app in a separate thread to avoid blocking ROS2
    flask_thread = Thread(target=run_flask_app)
    flask_thread.start()

    # ROS2 spin loop
    try:
        rclpy.spin(ros2_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        ros2_node.destroy_node()
        rclpy.shutdown()
        flask_thread.join()


if __name__ == "__main__":
    main()
