import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time


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
            self.get_logger().info(f'{len(self.waypoints)} waypoints from config.')
            
        self.navigator = BasicNavigator()
        
        # AMCL
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = -5.689477920532227  
        initial_pose.pose.position.y = 9.327399253845215 
        initial_pose.pose.orientation.z = 0.006001472473144531
        initial_pose.pose.orientation.w = 1.0 
        
        
        self.get_logger().info('Waiting for AMCL particle cloud')
        while not self.navigator.initial_pose_pub.get_subscription_count() > 0:
            self.get_logger().info('Waiting for AMCL node to subscribe to /initialpose topic')
            time.sleep(1.0)
        
        self.navigator.setInitialPose(initial_pose)
        
        # Nav Setup: Wait for Nav2 to be fully active
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 activate. Starting patrol setup')

        self.start_patrol()
        
    def start_patrol(self):
        '''
        Prepares  the poses and sends the goal to Nav2

        '''
        while rclpy.ok():
            for i, wp in enumerate(self.waypoints):
                self.get_logger().info(f'++ Going to Waypoint {i+1}: x={wp["x"]}, y={wp["y"]}')
                
                # Create goal
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = wp['x']
                goal_pose.pose.position.y = wp['y']
                goal_pose.pose.orientation.w = 1.0 # face foward

                # Go to goal
                self.navigator.goToPose(goal_pose)

                # Wait until robot gets to the point
                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    time.sleep(0.5)

                result = self.navigator.getResult()
                
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f'Waypoint {i+1} REACHED!')
                    time.sleep(1.0) # Wait 1 sec
                elif result == TaskResult.CANCELED:
                    self.get_logger().warn('Task was canceled.')
                    return
                elif result == TaskResult.FAILED:
                    self.get_logger().error(f'Failed to reach Waypoint {i+1}. Skipping...')
                    
            
            self.get_logger().info('Mission Passed') # Restarting patrol
            
def main(args=None):
        rclpy.init(args=args)
        node = SecurityPatrolNode()
        try:
            rclpy.spin(node) # For loop
        except KeyboardInterrupt:
            pass
        finally:
            # Destroy the node
            node.destroy_node()
            rclpy.shutdown()
            
if __name__ == '__main__':
    main()
        