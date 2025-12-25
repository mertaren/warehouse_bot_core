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
        
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for service")

    def joint_state_callback(self, msg):
        # Keep track of current robot joints
        self.current_joint_state = msg

    def get_ik_solution(self, x, y, z):
        """
        Requests Inverse Kinematics from MoveIt
        Returns: list of joint angles or None
        """
        if self.current_joint_state is None :
            self.get_logger().error("Joint state not recieved")
            return None
        
        # Construct GetPositionIK request
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.robot_state.joint_state = self.current_joint_state
        request.ik_request.avoid_collisions = True
        
        # Target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        
        # Quaternion
        pose_stamped.pose.orientation.w = 1.0 # Standart Orientaion
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        
        request.ik_request.pose_stamped = pose_stamped
        
        # Spin until complete
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response.error_code.val == 1:
            joint_positions = []
            target_joints = ['joint1', 'joint2', 'joint3', 'joint4']
            
            # Map the solution to the correct joint order
            for name in target_joints:
                if name in response.solution.joint_state.name:
                    idx = response.solution.joint_state.name.index(name)
                    pos = response.solution.joint_state.position[idx]
                    joint_positions.append(pos)
            
            self.get_logger().info(f"IK Solution Found: {joint_positions}")
            return joint_positions
        else:
            self.get_logger().error(f"IK Failed. Error Code: {response.error_code.val}")
            return None
        
    def execute_arm_trajectory(self, joint_positions):
        """
        Publishes the calculated joints to the controller
        """
        if not joint_positions:
            return

        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 4  # Allow 4 seconds for movement
        
        msg.points = [point]
        self.arm_publisher.publish(msg)
        
        # Blocking sleep to wait for movement
        time.sleep(4.5) 

    def operate_gripper(self, state):
        """
        state: 'open' or 'close'
        """
        goal = GripperCommand.Goal()
        goal.command.max_effort = 1.0
        
        if state == 'open':
            goal.command.position = 0.019
            self.get_logger().info("Opening gripper")
        elif state == 'close':
            goal.command.position = -0.010
            self.get_logger().info("Closing gripper")
        else:
            return

        # Wait for action server
        self._gripper_action_client.wait_for_server()
        
        # Send goal
        future = self._gripper_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f"Action completedf: {state}")
        time.sleep(1.0) 

def main(args=None):
    rclpy.init(args=args)
    node = ServicePickAndPlace()
    
    # Wait for joint states
    time.sleep(2)
    node.get_logger().info('Starting...')
    
    # [Waist, Shoulder, Elbow, Wrist]
    home_joints = [0.0, -1.0, 0.3, 0.7]
    node.execute_arm_trajectory(home_joints)

    # Open Gripper
    node.operate_gripper('open')

    # Approach Object
    node.get_logger().info("Approaching object...")
    approach_sol = node.get_ik_solution(0.25, 0.0, 0.20)
    if approach_sol:
        node.execute_arm_trajectory(approach_sol)
    else:
        node.get_logger().error("Approach Failed")
        return

    # Grasp
    node.get_logger().info("Lowering to grasp")
    grasp_sol = node.get_ik_solution(0.25, 0.0, 0.12)
    if grasp_sol:
        node.execute_arm_trajectory(grasp_sol)
    
    # Close Gripper
    node.operate_gripper('close')

    # Retreat
    node.get_logger().info("Retreating")
    retreat_sol = node.get_ik_solution(0.25, 0.0, 0.25)
    if retreat_sol:
        node.execute_arm_trajectory(retreat_sol)
    
    # Go home
    node.execute_arm_trajectory(home_joints)
    
    node.get_logger().info('Finished')
    
    node.destroy_node()
    rclpy.shutdown()
    
    node.get_logger().info('Closing')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()