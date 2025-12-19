import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from enum import Enum

# Defining states
class PatrolState(Enum):
    IDLE = 0 # On hold
    NAVIGATING = 1 # On the move
    SCANNING = 2 # Scanning

class SecurityPatrolNode(Node):
    def __init__(self):
        super().__init__('security_patrol_node')
        
        # Define params
        self.declare_parameter('waypoint__x', [0.0])
        self.declare_parameter('waypoint__y', [0.0])
        
        self.get_logger().info('Initializing security patrol node.')
        
        # Read params from yaml file
        wp_x = self.get_parameter('waypoint__x').get_parameter_value().double_array_value
        wp_y = self.get_parameter('waypoint__y').get_parameter_value().double_array_value
        
        # x =[1, 2] --> [{'x':1}] transfrom
        if len(wp_x) != len(wp_y):
            self.get_logger().error('x and y cords list must have the same lenght')
            self.waypoints = []
        else:
            self.waypoints = [{'x': x, 'y': y} for x, y in zip(wp_x, wp_y)]
            #self.get_logger().info(f'{len(self.waypoints)} waypoints from config.')
            
        self.navigator = BasicNavigator()
        # self.setup_initial_pose() # Assume initial pose is already set by Rviz/Sim startup if needed remove the '#'
        
        # State machine initialization
        self.state = PatrolState.IDLE
        self.current_wp_index = 0
        
        # Check every 0.5 sec
        self.timer = self.create_timer(0.5, self.fms_loop)
        self.get_logger().info('Patrol system ready...')
        
        # Starting navigation
        self.start_navigation()

    """
    def setup_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -5.69 
        initial_pose.pose.position.y = 9.32
        initial_pose.pose.orientation.z = 0.006
        initial_pose.pose.orientation.w = 1.0 
        
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
    """

    def start_navigation(self):
        '''
        Runs every 0.5 seconds

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
        self.state = PatrolState.NAVIGATING

    def start_scanning(self):
        self.get_logger().info(' Area Reached Scanning 360°')
        # Spin 360 degrees = 6.28 radians
        self.navigator.spin(spin_dist=6.28, time_allowance=10) 
        self.state = PatrolState.SCANNING


    def fms_loop(self):
        """
        Main finite state machine loop
        """
        # Robot is moving to a WP
        if self.state == PatrolState.NAVIGATING:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('WP Reached. Initiating Security Scan...')
                    self.start_scanning()
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        