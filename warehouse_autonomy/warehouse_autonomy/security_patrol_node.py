import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data 
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from enum import Enum

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# Defining states
class PatrolState(Enum):
    IDLE = 0 # On hold
    NAVIGATING = 1 # On the move
    SCANNING = 2 # Scanning

class SecurityPatrolNode(Node):
    def __init__(self):
        super().__init__('security_patrol_node')
        
        # Config
        self.declare_parameter('waypoint__x', [0.0])
        self.declare_parameter('waypoint__y', [0.0])
        
        wp_x = self.get_parameter('waypoint__x').get_parameter_value().double_array_value
        wp_y = self.get_parameter('waypoint__y').get_parameter_value().double_array_value
        
        if len(wp_x) != len(wp_y):
            self.get_logger().error('x and y cords list must have the same lenght')
            self.waypoints = []
        else:
            self.waypoints = [{'x': x, 'y': y} for x, y in zip(wp_x, wp_y)]
            #self.get_logger().info(f'{len(self.waypoints)} waypoints from config.')
            
        self.navigator = BasicNavigator()
        
        # Vision setup
        
        # Bridge object
        self.bridge = CvBridge()
        
        # Model loading
        self.model = YOLO('yolov8n.pt')
        
        # Define subscriber
        self.create_subscription(
            Image,
            '/pi_camera/image_raw',
            self.camera_callback,
            qos_profile_sensor_data) # Best practice
        self.get_logger().info('Camera online!')
        
        # State machine initialization
        self.state = PatrolState.IDLE
        self.current_wp_index = 0
        
        self.timer = self.create_timer(0.5, self.fms_loop)
        self.get_logger().info('Patrol system ready...')
        
        # Trigger first move
        self.start_navigation()

    def camera_callback(self, msg):
        try:
            # Convert ROS rgb Image to cv bgr image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            results = self.model(cv_image, verbose=False)
            
            annotated_frame = results[0].plot()
            
            # Show image
            cv2.imshow('Camera Feed', annotated_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error: {e} ")
            
    def start_navigation(self):
        '''
        Sends goal to Nav2 and updates state

        '''
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info('MISSION PASSED! Restarting Patrol...')
            self.current_wp_index = 0 # Loop back to start
        
        wp = self.waypoints[self.current_wp_index]
        self.get_logger().info(f'Moving to WP {self.current_wp_index + 1} ({wp["x"]}, {wp["y"]})')
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = wp['x']
        goal_pose.pose.position.y = wp['y']
        goal_pose.pose.orientation.w = 1.0 

        self.navigator.goToPose(goal_pose)
        # State update
        self.state = PatrolState.NAVIGATING

    def start_scanning(self):
        self.get_logger().info(' Area Reached Scanning 360°')
        # Spin 360 degrees = 6.28 radians
        self.navigator.spin(spin_dist=6.28, time_allowance=10) 
        # State update
        self.state = PatrolState.SCANNING

    def fms_loop(self):
        """
        Main finite state machine loop
        Checks status every 0.5s and decide next step
        """
        # Robot is moving to a WP
        if self.state == PatrolState.NAVIGATING:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('WP Reached. Initiating Security Scan...')
                    self.start_scanning() # Navigation done switch scanning
                else:
                    self.get_logger().error('Navigation Failed. Retrying same WP...')
                    self.start_navigation() # Retry logic

        # Robot is scanning
        elif self.state == PatrolState.SCANNING:
            if self.navigator.isTaskComplete():
                self.get_logger().info('Scan Complete. Area Secure.')
                self.current_wp_index += 1
                self.start_navigation() # Move to next point

def main(args=None):
    rclpy.init(args=args)
    node = SecurityPatrolNode()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        