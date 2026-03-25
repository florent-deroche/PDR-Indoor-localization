import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import paho.mqtt.client as mqtt 
import json
import math
from scipy.optimize import least_squares
from collections import deque 

# --- CONFIG ---
IP_ADRESS   = '10.120.2.231' # hostname -I 
PORT        = 1883
TOPIC       = 'PDR/robot/rssi'

# Anchor coordinates (X,Y) in meters
ANCHORS_POS = {
    'Anchor_1': (2.36, 5.92),
    'Anchor_2': (9.94, 0.25),
    'Anchor_3': (0.6, 1.5)
}

# Signal propagation model constants 
A_CONST = -40 # dB 1 meter 
N_CONST = 3.8  # Reflection Param      



class TrilaterationNode(Node):
    def __init__(self):
        self.previous_rssi_historic = deque(maxlen=50)
        super().__init__('ble_trilateration_node')
        
        self.position_publisher = self.create_publisher(PoseWithCovarianceStamped, 'ble_estimated_position', 10)
        
        
        

        # lire les msg mqtt
        self.mqtt_client = mqtt.Client()
        
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        self.get_logger().info(f"Connection to MQTT Broker IP : ({IP_ADRESS})...")
        self.mqtt_client.connect(IP_ADRESS, PORT, 60)
        self.mqtt_client.loop_start() 

    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0:
            self.get_logger().info("Connection succesful, waiting for RSSI data...")
            client.subscribe(TOPIC)
        else:
            self.get_logger().error(f"Connection failure : {reason_code}")

    def on_mqtt_message(self, client, userdata, msg):
        
        self.get_logger().info(f"Message brut reçu : {msg.payload.decode('utf-8')}")
        
        try:
            donnees = json.loads(msg.payload.decode('utf-8'))
            distances = {}
            
            
            for ancre, rssi in donnees.items():
                
                
                mesure_valide = (rssi != -100)
                if mesure_valide:
                    rssi_filtre = rssi
                else:
                    rssi_filtre = self.filter_list(ancre) # TO IMPLEMENT : update rssi to avoid -100 dB value (msg not caught)
                
                # log path distance model 
                distances[ancre] = 10 ** ((A_CONST - rssi_filtre) / (10 * N_CONST))

            self.previous_rssi_historic.append(donnees.copy())
            
            # coordinate calculation
            x, y = self.calculate_position(distances)
            
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
            self.get_logger().error(f"Error processing MQTT message: {e}")

    def filter_list(self, ancre):
        # On parcourt l'historique du plus récent au plus ancien
        for donnees_historique in reversed(self.previous_rssi_historic):
            if ancre in donnees_historique:
                rssi_precedent = donnees_historique[ancre]
                if rssi_precedent != -100:
                    return rssi_precedent
        
        # Sécurité : si l'historique est vide ou ne contient que des -100
        self.get_logger().warn(f"Aucun historique valide pour {ancre}, utilisation d'une valeur par défaut.")
        return -60 # Valeur d'un signal très lointain pour ne pas faire crasher les maths
    
    def error_function(self, guess, anchors, distances):
        errors = []
        for (ax, ay), d in zip(anchors, distances):
            calc_dist = math.sqrt((guess[0] - ax)**2 + (guess[1] - ay)**2)
            errors.append(calc_dist - d)
        return errors

    def calculate_position(self, distances_dict):
        anchors_list = []
        distances_list = []
        
        for ancre, dist in distances_dict.items():
            if ancre in ANCHORS_POS:
                anchors_list.append(ANCHORS_POS[ancre])
                distances_list.append(dist)
        
        initial_guess = [1.0, 0.5] # mettre centre de la pièce ? 
        result = least_squares(self.error_function, initial_guess, args=(anchors_list, distances_list))
        
        x_brut = result.x[0]
        y_brut = result.x[1]    
        return float(x_brut), float(y_brut)

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