#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_bot.srv import Savepoint
from geometry_msgs.msg import PoseStamped

import sys
import termios
import contextlib

import tty




class SaveCoordinatesClient(Node):

    

    def __init__(self):
        super().__init__('save_coordinates_client')
        self.cli = self.create_client(Savepoint, 'save_coordinates')
        self.create_subscription(PoseStamped, 'robot_position', self.get_pose_callback, 10)
        self.pose = None
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
            

    def send_coordinates(self,pose):
        request = Savepoint.Request()
        #request.poses = PoseStamped()
        if pose is not None:
            request.poses = pose
            print(type(pose))
        #request.poses.Pose = pose.Pose
        #print(type(pose))
        
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info('Coordinates saved successfully')
            else:
                self.get_logger().info('Failed to save coordinates')
        else:
            self.get_logger().info('Service call failed')

     

    def get_pose_callback(self, msg):
    # Traitez le message de localisation du robot ici
    # Stockez les coordonnées dans la variable pose
        self.pose = msg
        #SaveCoordinatesClient.pose1 = msg
        #request = Savepoint.Request()
        #request.pose = self.pose
        #print(self.pose)
        #return self.pose



def getch():
    """Lecture d'une seule touche sans afficher à l'écran."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
    



def main(args=None):
    rclpy.init(args=args)
    node = SaveCoordinatesClient()
    
    # Créez un objet PoseStamped avec les coordonnées du robot
    #pose = PoseStamped()
    # Set the values of pose
    while True:
        # Attendez l'appui sur la touche "s"
        key = getch()
        if key == 's':
            # Obtenez les coordonnées du robot à partir du nœud de localisation
            # Supposons que le message de localisation soit de type "PoseStamped"
            # Remplacez "nom_du_noeud_localisation" par le nom réel du nœud de localisation
            # et "topic_des_coordonnees" par le nom du topic où les coordonnées sont publiées
            #pose = node.create_subscription(PoseStamped, 'robot_position', node.get_pose_callback, 10)
            #print(node.pose)        
            # Envoyez les coordonnées pour sauvegarde
            node.send_coordinates(node.pose)    
        else:
            pass


    rclpy.spin(node)
    rclpy.shutdown()

    
    #node.destroy_node()

    

if __name__ == '__main__':
    main()
