#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
#import csv
import json

class GpsFileReader(Node):
    def __init__(self):
        super().__init__('gps_file_reader')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_vitual', 10)
        self.timer_ = self.create_timer(1.0, self.publish_gps)

        self.file_name_ = '/home/franklin/ALPHA_Path_Planning/Exports/Export_Fere1_10rg_south_31north31south.json' # i will use the json file 
        #self.gps_data_ = []

        self.gps_date_json = { "Path": [{"Alt": 0, "Lat": 0,"Long": 0}]}
        self.read_gps_data_from_file()
    
    def read_gps_data_from_file(self):

        with open(self.file_name_, "r") as read_file:    # configuration for json file
            json_reader = json.load(read_file)
            for v in range(1,10,len(json_reader['Path'])):
                self.gps_date_json["Path"].append({'Alt':json_reader["Path"][v]["Alt"],
                                                    'Lat':json_reader["Path"][v]["Lat"],
                                                    'Long': json_reader["Path"][v]["Long"]}) 


        #with open(self.file_name_, 'r') as csv_file:
        #    csv_reader = csv.reader(csv_file)
        #    for row in csv_reader:
        #        if len(row) == 2:
        #            self.gps_data_.append((float(row[0]), float(row[1])))

    #def publish_gps(self):
    #    if len(self.gps_data_) > 0:
    #        lat, lon = self.gps_data_.pop(0)
    #        msg = NavSatFix()
    #        msg.latitude = lat
    #        msg.longitude = lon
    #        self.publisher_.publish(msg)

    def publish_gps(self):
        msg = NavSatFix()
        if (len(self.gps_date_json["Path"]) > 1) : 
            for w in range(1,len(self.gps_date_json["Path"])):
                msg.altitude = float (self.gps_date_json['Path'][w]['Alt'])
                msg.latitude  = float(self.gps_date_json['Path'][w]['Lat'])
                msg.longitude = float(self.gps_date_json['Path'][w]['Long'])
                msg.header.frame_id = 'gps_vitual_link'
                self.publisher_.publish(msg)
        else:
            print('Pas acces d element dans le fichier')


        


def main(args=None):
    rclpy.init(args=args)
    node = GpsFileReader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()