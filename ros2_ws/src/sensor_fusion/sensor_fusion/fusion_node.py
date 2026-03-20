import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np

class KalmanFusionNode(Node):
    def __init__(self):
        super().__init__('kalman_fusion_node')

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, 
            'ble_estimated_position', 
            self.ble_callback, 
            10
        )
        self.publisher = self.create_publisher(PoseStamped, 'fused_pose', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
      

        self.last_odom_x = None
        self.last_odom_y = None

        self.X = np.zeros((2, 1))
        self.P = np.eye(2) * 1.0 
        self.Q = np.eye(2) * 0.05 
        self.R = np.eye(2) * 1.69 

        self.timer = self.create_timer(0.1, self.prediction_step)
        self.get_logger().info("Kalman Fusion Node Started, Waiting for odometry...")

    def prediction_step(self):
        try:
            now = rclpy.time.Time()
            # On écoute juste de combien le robot a bougé depuis la dernière fois
            trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', now)
            
            curr_odom_x = trans.transform.translation.x
            curr_odom_y = trans.transform.translation.y

            if self.last_odom_x is None:
                self.last_odom_x = curr_odom_x
                self.last_odom_y = curr_odom_y
                return

            dx = curr_odom_x - self.last_odom_x
            dy = curr_odom_y - self.last_odom_y

            self.X[0, 0] += dx
            self.X[1, 0] += dy
            
            self.P = self.P + self.Q

            self.last_odom_x = curr_odom_x
            self.last_odom_y = curr_odom_y

            self.publish_fused_pose()

        except TransformException:
            pass

    def ble_callback(self, msg):
        ble_x = msg.pose.pose.position.x
        ble_y = msg.pose.pose.position.y

        if self.last_odom_x is None or (self.X[0, 0] == 0 and self.X[1, 0] == 0):
            self.X[0, 0] = ble_x
            self.X[1, 0] = ble_y
            return 

        Z = np.array([[ble_x], [ble_y]])
        Y = Z - self.X
        S = self.P + self.R
        K = np.dot(self.P, np.linalg.inv(S))
        self.X = self.X + np.dot(K, Y)
        
        I = np.eye(2)
        self.P = np.dot((I - K), self.P)

    def publish_fused_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # On publie la position dans notre propre repère d'ancres !
        msg.header.frame_id = 'ble_origin' 
        
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