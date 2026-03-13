import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import paho.mqtt.client as mqtt 
import json
import math
from scipy.optimize import least_squares

# --- CONFIG ---
IP_ADRESS   = '10.120.2.235' #mqtt address
PORT        = 1883
TOPIC       = 'PDR/robot/rssi'

# Anchor coordinates (X,Y) in meters
ANCHORS_POS = {
    'Anchor_1': (2.15, 0.0),
    'Anchor_2': (1.05, 1.3),
    'Anchor_3': (0.0, 0.0)
}

# Signal propagation model constants 
A_CONST = -33.0 
N_CONST = 3.8   

# 1D Kalman filter to smooth RSSI data 
class KalmanFilter1D:
    def __init__(self, q, r, p, initial_value):
        self.q = q
        self.r = r
        self.p = p
        self.x = initial_value

    def process(self, measurement, is_valid):
        self.p = self.p + self.q
        if is_valid:
            k = self.p / (self.p + self.r)
            self.x = self.x + k * (measurement - self.x)
            self.p = (1 - k) * self.p
        return self.x

# ROS2 Node for trilateration based on MQTT RSSI data
class TrilaterationNode(Node):
    def __init__(self):
        super().__init__('ble_trilateration_node')
        
        # Publisher node creation for estimated position
        self.position_publisher = self.create_publisher(Point, 'ble_estimated_position', 10)
        
        # Filter init for each anchor
        self.filtres_ancres = {
            'Anchor_1': KalmanFilter1D(q=0.05, r=25.0, p=1.0, initial_value=-60.0),
            'Anchor_2': KalmanFilter1D(q=0.05, r=25.0, p=1.0, initial_value=-60.0),
            'Anchor_3': KalmanFilter1D(q=0.05, r=25.0, p=1.0, initial_value=-60.0)
        }

        # MQTT Config 
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        self.get_logger().info(f"Connection to MQTT Broker IP : ({IP_ADRESS})...")
        self.mqtt_client.connect(IP_ADRESS, PORT, 60)
        self.mqtt_client.loop_start() 

    def on_mqtt_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            self.get_logger().info("Connection succesful, waiting for RSSI data...")
            client.subscribe(TOPIC)
        else:
            self.get_logger().error(f"Connection failure : {reason_code}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            # JSON parsing of the incoming message
            donnees = json.loads(msg.payload.decode('utf-8'))
            distances = {}
            
            # Processing each anchor's RSSI with Kalman filter and converting to distance
            for ancre, rssi in donnees.items():
                mesure_valide = (rssi != -100)
                rssi_filtre = self.filtres_ancres[ancre].process(measurement=rssi, is_valid=mesure_valide)
                
                
                distances[ancre] = 10 ** ((A_CONST - rssi_filtre) / (10 * N_CONST))

            
            x, y = self.calculate_position(distances)
            
            # Publishing on ROS Topic 
            position_msg = Point()
            position_msg.x = x
            position_msg.y = y
            position_msg.z = 0.0 
            
            self.position_publisher.publish(position_msg)
            self.get_logger().debug(f"Published position : X={x:.2f}, Y={y:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")


    # TO DO : Change this or remove it if you don't need it, it's just an example of a method that could be used in the error function for least squares optimization
    def error_function(self, guess, anchors, distances):
        """Least squares error function for trilateration"""
        errors = []
        for (ax, ay), d in zip(anchors, distances):
            calc_dist = math.sqrt((guess[0] - ax)**2 + (guess[1] - ay)**2)
            errors.append(calc_dist - d)
        return errors

    def calculate_position(self, distances_dict):
        """Calculate the position (X,Y) by minimizing the error on the 3 distances"""
        anchors_list = []
        distances_list = []
        
        for ancre, dist in distances_dict.items():
            if ancre in ANCHORS_POS:
                anchors_list.append(ANCHORS_POS[ancre])
                distances_list.append(dist)
        
        initial_guess = [1.0, 0.5] 
        result = least_squares(self.error_function, initial_guess, args=(anchors_list, distances_list))
        
        x_brut = result.x[0]
        y_brut = result.x[1]
        
        # Bias correction based on empirical calibration
        x_corrige = x_brut - 0.882
        y_corrige = y_brut - 0.124
        
        return x_corrige, y_corrige

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
