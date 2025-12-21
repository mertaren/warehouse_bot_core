import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

# ROS2 and MoveIt Messages
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState

class ServicePickAndPlace(Node):
    def __init__(self):
        super().__init__('service_pick_place_node')
        
        # COMMUNICATUON
        
        # Arm Controller
        self.arm_publisher = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10
        )
        
        # Gripper Controller
        self._gripper_action_client = ActionClient(
            self, 
            GripperCommand, 
            '/gripper_controller/gripper_cmd'
        )

        # MoveIt IK Service
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # Joint State Listener
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.current_joint_state = None

        self.get_logger().info('Waiting for services')
        # TODO: Add logic to wait for /compute_ik service availability

    def joint_state_callback(self, msg):
        # Keep track of current robot joints
        # This is needed to prevent IK jumps/collisions
        self.current_joint_state = msg

    def get_ik_solution(self, x, y, z):
        """
        Requests Inverse Kinematics from MoveIt
        Returns: list of joint angles or None
        """
        # TODO: Check if self.current_joint_state is valid

        # TODO: Call service synchronously or asynchronously
        
        pass

    def execute_arm_trajectory(self, joint_positions):
        """
        Publishes the calculated joints to the controller
        """
        # TODO: Create JointTrajectory message
        #  Fill joint_names
        #  JointTrajectoryPoint
        
        # TODO: Publish message
        pass

    def operate_gripper(self, state):
        """
        state: 'open' or 'close'
        """
        # TODO: Define values
        # Open: 0.019
        # Close: -0.010 : Tested ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: -0.010, max_effort: 1.0}}"
        
        # TODO: Send goal to action server and wait for result
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ServicePickAndPlace()
    
    node.get_logger().info('Starting Sequence')

    # TODO: Execution flow
    
   
    
    node.get_logger().info('Sequence Finished')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()