import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np

class KalmanFusionNode(Node):
    def __init__(self):
        super().__init__('kalman_fusion_node')

        # /--- Subscriptions and publications ---/
        # Listen to the position calculated by the 'ble_localization' package
        self.subscription = self.create_subscription(
            Point, 
            'ble_estimated_position', 
            self.ble_callback, 
            10
        )
        # Publish the filtered pose for RViz2
        self.publisher = self.create_publisher(PoseStamped, 'fused_pose', 10)

        # /--- Odometry management (TF) ---/
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_odom_x = None
        self.last_odom_y = None

        # /--- Kalman Filter Configuration ---/
        # Intial state: X = [x, y]^T
        self.X = np.zeros((2, 1))
        
        # Uncertainty Matrix (P): Average confidence 
        self.P = np.eye(2) * 1.0 
        
        # Model noise (Q): Confidence in odometry (very reliable in the short term)
        self.Q = np.eye(2) * 0.05 
        
        # Noise of measurement (R): Confidence in BLE.
        # Based on your real tests: error of ~1.3m -> Variance = 1.3^2 = 1.69
        self.R = np.eye(2) * 1.69 

        # --- Prediction loop (10 Hz) ---
        self.timer = self.create_timer(0.1, self.prediction_step)
        self.get_logger().info("Kalman Fusion Node Started, Waiting for odometry...")

    def prediction_step(self):
        """ Prediction step based on Odometry """
        try:
            # Get the real displacement of the robot via odometry
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', now)
            
            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y

            # Initialization at the first pass
            if self.last_odom_x is None:
                self.last_odom_x = curr_x
                self.last_odom_y = curr_y
                self.X[0, 0] = curr_x
                self.X[1, 0] = curr_y
                return

            # Calculate the displacement vector (Delta odom)
            dx = curr_x - self.last_odom_x
            dy = curr_y - self.last_odom_y

            # --- Kalman prediciton ---
            self.X[0, 0] += dx
            self.X[1, 0] += dy
            
            # P = P + Q 
            self.P = self.P + self.Q

        
            self.last_odom_x = curr_x
            self.last_odom_y = curr_y

    
            self.publish_fused_pose()

        except TransformException:
    
            pass

    def ble_callback(self, msg):
        """ Update based on BLE measurement """
        if self.last_odom_x is None:
            return 

        # Measurement vector Z
        Z = np.array([[msg.x], [msg.y]])
        
        
        # Y = Z - X (Innovation)
        Y = Z - self.X
        
        # Covariance of the Innovation (S = P + R)
        S = self.P + self.R
        
        # Kalman Gain (K = P * S^-1)
        K = np.dot(self.P, np.linalg.inv(S))
        
        # State update (X = X + K * Y)
        self.X = self.X + np.dot(K, Y)
        
        # Covariance update (P = (I - K) * P)
        I = np.eye(2)
        self.P = np.dot((I - K), self.P)

        self.get_logger().info(f'BLE correction applied : {np.trace(self.P):.2f}')

    def publish_fused_pose(self):
        """ Edit and publish data for ROS2 Ecosystem. """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom' # Global reference frame
        
        msg.pose.position.x = float(self.X[0, 0])
        msg.pose.position.y = float(self.X[1, 0])
        msg.pose.position.z = 0.0
        
        
        msg.pose.orientation.w = 1.0 
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
