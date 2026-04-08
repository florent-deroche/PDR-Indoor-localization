import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import paho.mqtt.client as mqtt 
import json
import math
from scipy.optimize import least_squares
from collections import deque 
 
# --- CONFIG ---
IP_ADRESS   = '10.120.2.231'
PORT        = 1883
TOPIC       = 'PDR/robot/rssi'
 
# Anchor Coordinates (X, Y ,Z)
ANCHORS_POS = {
    'Anchor_1': (2.36, 5.92, 0.88),
    'Anchor_2': (9.94, 0.25, 1.30),
    'Anchor_3': (0.6, 1.5, 1.30)
}

ROBOT_Z = 0.29
# Signal Propagation parameters 
A_CONST = -45  # 1 meter distance's RSSI 
N_CONST = 2  # Measured manually
 
# EMA Filter Intensity
EMA_ALPHA = 0.2
 
# Minimum Anchor number for position computing
MIN_ANCHORS = 3
 
 
class TrilaterationNode(Node):
    def __init__(self):
        self.previous_rssi_historic = deque(maxlen=50)
        super().__init__('ble_trilateration_node')
        
        self.position_publisher = self.create_publisher(PoseWithCovarianceStamped, 'ble_estimated_position', 10)
 
        
        # EMA Filter for each anchor : stores smoothed RSSO for each anchor independently. 
        self.rssi_ema = {}
 
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        self.get_logger().info(f"Connexion au broker MQTT ({IP_ADRESS})...")
        self.mqtt_client.connect(IP_ADRESS, PORT, 60)
        self.mqtt_client.loop_start() 
 
    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            self.get_logger().info("Connexion MQTT réussie, en attente de données RSSI...")
            client.subscribe(TOPIC)
        else:
            self.get_logger().error(f"Échec de connexion MQTT : code {reason_code}")
 
    def on_mqtt_message(self, client, userdata, msg):
        try:
            donnees = json.loads(msg.payload.decode('utf-8'))
            distances = {}
            
            for ancre, rssi in donnees.items():
 
                # Getting Valid RSSI 
                if rssi != -100:
                    rssi_brut = rssi
                else:
                    rssi_brut = self.get_last_valid_rssi(ancre)
                    if rssi_brut is None:
                        self.get_logger().warn(f"Aucune donnée valide pour {ancre}, ancre ignorée.")
                        continue
 
                
                if ancre not in self.rssi_ema:
                    self.rssi_ema[ancre] = float(rssi_brut)  # initialisation directe
                else:
                    self.rssi_ema[ancre] = EMA_ALPHA * rssi_brut + (1.0 - EMA_ALPHA) * self.rssi_ema[ancre]
 
                rssi_filtre = self.rssi_ema[ancre]
 
                # Log Path distance Model 
                distances[ancre] = 10 ** ((A_CONST - rssi_filtre) / (10 * N_CONST))
 
            self.previous_rssi_historic.append(donnees.copy())
 
            # Diagnostic log 
            log_parts = [
                f"{a}: {self.rssi_ema.get(a, '?'):.1f}dBm → {distances.get(a, '?'):.2f}m"
                for a in donnees if a in distances
            ]
            self.get_logger().info(" | ".join(log_parts))
 
            # Publish only if all anchors are visible 
            if len(distances) < MIN_ANCHORS:
                self.get_logger().warn(f"Seulement {len(distances)} ancre(s) visible(s), position non publiée.")
                return
 
            x, y = self.calculate_position(distances)
            self.get_logger().info(f"Position estimée → X={x:.2f}m, Y={y:.2f}m")
 
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'ble_origin' 
            pose_msg.pose.pose.position.x = x
            pose_msg.pose.pose.position.y = y
            pose_msg.pose.pose.position.z = 0.0 
            pose_msg.pose.pose.orientation.w = 1.0
            pose_msg.pose.covariance[0] = 0.5
            pose_msg.pose.covariance[7] = 0.5
            self.position_publisher.publish(pose_msg)
 
        except Exception as e:
            self.get_logger().error(f"Erreur de traitement du message MQTT : {e}")
 
    def get_last_valid_rssi(self, ancre):
        """Return latest valid RSSI measurelment for each anchor"""
        for donnees_historique in reversed(self.previous_rssi_historic):
            if ancre in donnees_historique and donnees_historique[ancre] != -100:
                return float(donnees_historique[ancre])
        return None
 
    def error_function(self, guess, anchors, distances):
        errors = []
        for (ax, ay, az), d in zip(anchors, distances):
            calc_dist = math.sqrt((guess[0] - ax)**2 + (guess[1] - ay)**2+(ROBOT_Z - az)**2)
            errors.append(calc_dist - d)
        return errors
 
    def calculate_position(self, distances_dict):
        anchors_list = []
        distances_list = []
        
        for ancre, dist in distances_dict.items():
            if ancre in ANCHORS_POS:
                anchors_list.append(ANCHORS_POS[ancre])
                distances_list.append(dist)
 
        # Initial guess = centroïd of visible anchors
        # used to avoid local minimums of least squares resolution
        x0 = sum(a[0] for a in anchors_list) / len(anchors_list)
        y0 = sum(a[1] for a in anchors_list) / len(anchors_list)
        limites = ([-2.0, -2.0], [12.0, 8.0])
        
        result = least_squares(
            self.error_function,
            [x0, y0],
            bounds=limites,
            args=(anchors_list, distances_list)
        )
        
        return float(result.x[0]), float(result.x[1])
 
 
def main():
    rclpy.init()
    node = TrilaterationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()
