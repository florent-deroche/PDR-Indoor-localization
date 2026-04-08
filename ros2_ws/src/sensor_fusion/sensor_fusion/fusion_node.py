import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformException, Buffer, TransformListener, TransformBroadcaster
import numpy as np
import math

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
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_odom_x = None
        self.last_odom_y = None
        
        self.theta_offset = 0.0 
        self.current_yaw = 0.0 

        # Kalman Filter (X, Y)
        self.X = np.zeros((2, 1))
        self.P = np.eye(2) * 4.0
        self.Q = np.eye(2) * 0.005
        self.R = np.eye(2) * 25

        # Initialisation flag 
        self.is_initialized = False

        self.timer = self.create_timer(0.1, self.prediction_step)
        self.get_logger().info("Kalman Fusion Node démarré, en attente d'odométrie...")

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_quaternion_from_yaw(self, yaw):
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }

    def prediction_step(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            
            curr_odom_x = trans.transform.translation.x
            curr_odom_y = trans.transform.translation.y
            
            odom_yaw = self.get_yaw_from_quaternion(trans.transform.rotation)
            self.current_yaw = odom_yaw + self.theta_offset

            if self.last_odom_x is None:
                self.last_odom_x = curr_odom_x
                self.last_odom_y = curr_odom_y
                return

            dx_odom = curr_odom_x - self.last_odom_x
            dy_odom = curr_odom_y - self.last_odom_y

            dx_global = dx_odom * math.cos(self.theta_offset) - dy_odom * math.sin(self.theta_offset)
            dy_global = dx_odom * math.sin(self.theta_offset) + dy_odom * math.cos(self.theta_offset)

            # Kalman Prediction 
            self.X[0, 0] += dx_global
            self.X[1, 0] += dy_global
            self.P = self.P + self.Q

            self.last_odom_x = curr_odom_x
            self.last_odom_y = curr_odom_y

            self.publish_state(curr_odom_x, curr_odom_y)

        except TransformException:
            pass

    def ble_callback(self, msg):
        ble_x = msg.pose.pose.position.x
        ble_y = msg.pose.pose.position.y

        
        # Clean Init at the first valid BLE Measure
        if not self.is_initialized:
            self.X[0, 0] = 0 # we start in (0,O)
            self.X[1, 0] = 0
            self.is_initialized = True
            self.get_logger().info(f"Kalman initialisé à la position BLE : ({ble_x:.2f}, {ble_y:.2f})")
            return

        # Kalman Update 
        Z = np.array([[ble_x], [ble_y]])
        Y = Z - self.X
        S = self.P + self.R
        K = np.dot(self.P, np.linalg.inv(S))
        self.X = self.X + np.dot(K, Y)
        
        I = np.eye(2)
        self.P = np.dot((I - K), self.P)

    def publish_state(self, odom_x, odom_y):
        now = self.get_clock().now().to_msg()
        q_yaw = self.get_quaternion_from_yaw(self.current_yaw)

        
        # Publishing fused_pose in ble_origin (equals to map thanks to static tf in launch file)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'ble_origin' 
        pose_msg.pose.position.x = float(self.X[0, 0])
        pose_msg.pose.position.y = float(self.X[1, 0])
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = q_yaw['x']
        pose_msg.pose.orientation.y = q_yaw['y']
        pose_msg.pose.orientation.z = q_yaw['z']
        pose_msg.pose.orientation.w = q_yaw['w']
        self.publisher.publish(pose_msg)

        
        
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'ble_origin'
        t.child_frame_id = 'odom'
        tx = float(self.X[0, 0]) - (odom_x * math.cos(self.theta_offset) - odom_y * math.sin(self.theta_offset))
        ty = float(self.X[1, 0]) - (odom_x * math.sin(self.theta_offset) + odom_y * math.cos(self.theta_offset))
        t.transform.translation.x = tx
        t.transform.translation.y = ty  
        t.transform.translation.z = 0.0
        q_offset = self.get_quaternion_from_yaw(self.theta_offset)
        t.transform.rotation.x = q_offset['x']
        t.transform.rotation.y = q_offset['y']
        t.transform.rotation.z = q_offset['z']
        t.transform.rotation.w = q_offset['w']
        #self.tf_broadcaster.sendTransform(t) --> commented so our system doesn't override slam's calculations 


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
