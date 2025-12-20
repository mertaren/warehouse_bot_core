import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class ArmTestNode(Node):
    def __init__(self):
        super().__init__('arm_test_node')
        
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_contreoller/joint_trajectory',
            10)
        
        # For OpenManipulator 4 joints
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
    def send_command(self, positions, duration=2.0):
        
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"Commands: {positions}")

def main(args=None):
    rclpy.init(args=args)
    node = ArmTestNode()
    
    try:
        time.sleep(1)
        
        # Home position
        print('MOVING HOME')
        node.send_command([0.0, -1.0, 0.3, 0.7], duration=2)
        time.sleep(3)
        
        # Grip position
        print('GRIP POSE')
        node.send_command([0.0, 0.5, 0.5, 0.0], duration=2)
        time.sleep(3)
        
        print('test complete')
    
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()