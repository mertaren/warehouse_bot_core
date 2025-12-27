import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.counter = 0
        
        self.timer = self.create_timer(1.0, self.publish_pose)
        self.get_logger().info('Initial Pose Publisher Preparing...')

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
       
        msg.pose.pose.position.x = 0.1578
        msg.pose.pose.position.y = -0.17247
        msg.pose.pose.position.z = 0.0
        
        
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.00035
        msg.pose.pose.orientation.w = 0.99999

        # Covariance matris
        msg.pose.covariance = [
            0.1991521, 0.0004791921160253107, 0.0, 0.0, 0.0, 0.0,
            0.0004791921160253107, 0.23625413334343975, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05757644606312473
        ]

        self.publisher_.publish(msg)
        self.counter += 1
        
        if self.counter >= 3:
            self.get_logger().info('Initial Pose Sent Closing..')
            # Execute 
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()