import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from copy import deepcopy


class SecurityPatrolNode(Node):
    def __init__(self):
        super().__init__('security_patrol_node')
        
        self.get_logger().info('Initializing security patrol node.')
        
        self.navigator = BasicNavigator()
        
        # Nav Setup: Wait for Nav2 to be fully active
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 activate. Starting patrol setup')
        
        # Route ( Futue : We'll load this from YAML)
        self.raw_route = [
            [10.4, -0.66],
            [20.2, -1.66],
            [17.56, -7.32]
        ]
        
        self.is_patrolling = False
        self.start_patrol()
        
    def start_patrol(self):
        '''
        Prepares  the poses and sends the goal to Nav2

        '''
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0 # Facing foward
        for pt in self.raw_route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))
        
        self.get_logger().info(f'Starting patrol with{len(route_poses)} waypoints')
        
        # Send goal
        self.navigator.goThroughPoses(route_poses)
        self.is_patrolling = True
        
        # Create timer because this allows the node to do other things
        self.timer = self.create_timer(1.0, self.patrol_feedback_callback)
        
    def patrol_feedback_callback(self):
        '''           
         Periodic check of the navigation status
        
        '''
        if not self.is_patrolling:
            return
        
        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            return
        
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Route complete! Restarting...')
            self.raw_route.revese()
            self.start_patrol() # Loop behavior
            
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Security route was canceled') 
            self.is_patrolling = False # Return to home
        
        elif result == TaskResult.FAILED:
            self.get_logger().info('Navigation failed') 
            self.is_patrolling = False
            self.start_patrol
            
def main(args=None):
        rclpy.init(args=args)
        try:
            node = SecurityPatrolNode()
            rclpy.spin(node) # For loop
        except KeyboardInterrupt:
            pass
        finally:
            # Destroy the node
            rclpy.shutdown()
            
if __name__ == '__main__':
    main()
        