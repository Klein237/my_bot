#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_bot.srv import Savepoint
from geometry_msgs.msg import PoseStamped
import json

class SaveCoordinatesServer(Node):

    def __init__(self):
        super().__init__('save_coordinates_server')
        self.srv = self.create_service(Savepoint, 'save_coordinates', self.save_coordinates_callback)
        #self.pose = PoseStamped()
        self.file_name = '/home/franklin/ws/src/my_bot/script/test11.json'
        

    def save_coordinates_callback(self, request, response):
        # Obtenez les coordonnées du robot à partir de la demande
        
        pose = request.poses
        
        
        #self.pose = pose

        # Ici, vous pouvez effectuer des opérations de sauvegarde des coordonnées sur la carte
        self.save_pose_stamped_to_json(self.file_name, pose)
        # Renvoyer la réponse indiquant que la sauvegarde a réussi
        response.success = True
        return response
    
    
    
    def save_pose_stamped_to_json(self,file_path, pose_stamped):
        with open("/home/franklin/ws/src/my_bot/script/test5.json", "r") as read_file:
            data = json.load(read_file)
        
        
        if pose_stamped is not None:

            data["Path"].append({
                'pose': {
                    'position': {
                        'x': pose_stamped.pose.position.x,
                        'y': pose_stamped.pose.position.y,
                        'z': pose_stamped.pose.position.z
                    },
                    'orientation': {
                        'x': pose_stamped.pose.orientation.x,
                        'y': pose_stamped.pose.orientation.y,
                        'z': pose_stamped.pose.orientation.z,
                        'w': pose_stamped.pose.orientation.w
                    }
                },
                'header': {
                    'frame_id': pose_stamped.header.frame_id
                }
            })
            


        
        with open(file_path, 'a') as file:
            json.dump(data, file)
            file.write('\n') 

def main(args=None):
    rclpy.init(args=args)
    #file_name = '/home/franklin/ws/src/my_bot/script/test.json'
    node = SaveCoordinatesServer()
    #node.save_pose_stamped_to_json(file_name,node.pose)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
