# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Ce noeud récolte les positions estimées avec le BLE seul, avec le filtre de
Kalman (BLE + odométrie) et avec le SLAM (LiDAR + odométrie), puis les
enregistre dans un fichier CSV.

Il faut que les 3 topics publient activement pour que les données soient
enregistrées. Il faut donc impérativement lancer le SLAM.


commande pour lancer :
ros2 run data_collection data_collector

Les résultats sont enregistrés dans positions.csv dans le répertoire
/PDR-Indoor-localization-main
"""

import rclpy
from rclpy.node import Node
import message_filters
import csv
import os
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs

class DataCollectionNode(Node):

    def __init__(self):
        super().__init__('data_collection_node')

        self.csv_filename = '../positions.csv'
        self.init_csv()

        # Position estimée uniquement avec le BLE
        self.sub_ble = message_filters.Subscriber(
            self, 
            PoseWithCovarianceStamped, 
            'ble_estimated_position'
        )

        # Position estimée par le filtre de Kalman
        self.sub_kalman = message_filters.Subscriber(
            self, 
            PoseStamped, 
            'fused_pose'
        )

        # ATTENTION: On n'écoute plus le topic /pose du SLAM !
        # On va chercher la position directement dans l'arbre TF.

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # On ne synchronise plus que le BLE et le Kalman
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_ble, self.sub_kalman],
            queue_size=10,
            slop=0.5  
        )

        self.ts.registerCallback(self.synchronized_callback)
        self.get_logger().info('Collecte des positions démarrée (Cible: ble_origin)')

    def init_csv(self):
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['t', 'x_ble', 'y_ble', 'x_kalman', 'y_kalman', 'x_slam', 'y_slam'])

    def synchronized_callback(self, ble_msg, kalman_msg):
        t = ble_msg.header.stamp.sec + ble_msg.header.stamp.nanosec * 1e-9
        
        ble_pose_stamped = PoseStamped()
        ble_pose_stamped.header = ble_msg.header
        ble_pose_stamped.pose = ble_msg.pose.pose

        # Le repère dans lequel on veut tout enregistrer
        target_frame = 'ble_origin'

        try:
            # 1. Conversion des coordonnées BLE (au cas où elles ne seraient pas déjà dans ble_origin)
            t_ble = self.tf_buffer.lookup_transform(
                target_frame,               
                ble_msg.header.frame_id,    
                rclpy.time.Time()           
            )
            ble_transformed = tf2_geometry_msgs.do_transform_pose(ble_pose_stamped.pose, t_ble)

            # 2. Conversion des coordonnées Kalman
            t_kalman = self.tf_buffer.lookup_transform(
                target_frame, 
                kalman_msg.header.frame_id, 
                rclpy.time.Time()
            )
            kalman_transformed = tf2_geometry_msgs.do_transform_pose(kalman_msg.pose, t_kalman)

            # 3. EXTRACTION DU SLAM VIA TF2 (La nouveauté !)
            # On demande la transformation entre l'origine et le robot
            t_robot = self.tf_buffer.lookup_transform(
                target_frame,       # Repère parent (Cible)
                'base_footprint',   # Repère enfant (La base du robot)
                rclpy.time.Time()
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Transformation TF impossible : {e}')
            return 

        # Extraction des coordonnées 
        x_ble = ble_transformed.position.x
        y_ble = ble_transformed.position.y
        
        x_kalman = kalman_transformed.position.x
        y_kalman = kalman_transformed.position.y
        
        # Pour le SLAM, la position est directement contenue dans la translation TF !
        x_slam = t_robot.transform.translation.x
        y_slam = t_robot.transform.translation.y

        # Écriture dans le CSV
        with open(self.csv_filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([t, x_ble, y_ble, x_kalman, y_kalman, x_slam, y_slam])
            
        self.get_logger().info(f'Positions enregistrées dans {target_frame} à t={t:.2f}')


def main(args=None):
    rclpy.init(args=args)
    data_collection_node = DataCollectionNode()
    try:
        rclpy.spin(data_collection_node)
    except KeyboardInterrupt:
        pass
    finally:
        data_collection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
