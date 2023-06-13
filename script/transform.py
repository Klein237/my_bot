#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class RobotPositionPublisher(Node):
    def __init__(self):
        super().__init__('robot_position_publisher')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher_ = self.create_publisher(PoseStamped, 'robot_position', 10)
        self.timer_ = self.create_timer(1.0, self.publish_position)

    def publish_position(self):
        try:
            # Vérifier si la transformation entre les repères "map" et "base_link" est possible
            if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                # Récupérer la transformation
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

                # Créer un message PoseStamped avec les coordonnées de la position
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = 'map'
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.pose.position.x = transform.transform.translation.x
                pose_msg.pose.position.y = transform.transform.translation.y
                pose_msg.pose.position.z = transform.transform.translation.z
                pose_msg.pose.orientation = transform.transform.rotation

                # Publier le message de position
                self.publisher_.publish(pose_msg)

            else:
                self.get_logger().warn('Transformation entre "map" et "base_link" non disponible')

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error('Erreur lors de la récupération de la transformation : %s' % str(e))


def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
