from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy
from copy import deepcopy

def main ():
    rclpy.init()
    
    navigator = BasicNavigator()
    
    # Route
    s_route = [
        [10.5, -0.77],
        [16.22, -0.77],
        [23.56, -2.5]
    ]
    
    # Initial poses
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 1.0
    # navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate
    navigator.waitUntilNav2Active()
    
    # do route until shutdown
    while rclpy.ok():
        
        # send route
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        for pt in s_route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))
        
        navigator.goThroughPoses(route_poses)
        
        
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            pass

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Route complete! Restarting...')
            s_route.reverse()
        elif result == TaskResult.CANCELED:
            print('Security route was canceled. Exiting.')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Navigation failed')
            print('Restarting...')

    exit(0)

if __name__ == '__main__':
    main()