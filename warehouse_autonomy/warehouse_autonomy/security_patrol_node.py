import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time


class SecurityPatrolNode(Node):
    def __init__(self):
        super().__init__('security_patrol_node')
        
        self.get_logger().info('Initializing security patrol node.')
        
        self.navigator = BasicNavigator()
        
        # Nav Setup: Wait for Nav2 to be fully active
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 activate. Starting patrol setup')
        
        # Define Route (Hardcoded for now)
        self.waypoints = [   
            {'x': 1.45, 'y': 6.64},    # Point A
            {'x': -3.50, 'y': -2.88},  # Point B
            {'x': 5.63, 'y': 2.07}   # Point C
        ]
        
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

                # Go to first goal
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
                    
            
            self.get_logger().info('Mission Passed')
            
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
        