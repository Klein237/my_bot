import unittest
import rclpy
import sys
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose, FollowWaypoints, ComputePathToPose, ComputePathThroughPoses
from nav2_msgs.srv import LoadMap, ClearEntireCostmap, GetCostmap

# Add script directory to sys.path to allow importing BasicNavigator
sys.path.append(os.path.join(os.path.dirname(__file__), '../script'))
from robot_navigator import BasicNavigator

class TestBasicNavigator(unittest.TestCase):
    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    def test_initial_pose_publisher_creation(self):
        basic_navigator = BasicNavigator()
        self.assertIsNotNone(basic_navigator.initial_pose_pub)
        self.assertEqual(basic_navigator.initial_pose_pub.topic_name, '/initialpose')
        self.assertEqual(basic_navigator.initial_pose_pub.msg_type, PoseWithCovarianceStamped)
        basic_navigator.destroy_node() # Clean up the node

    def test_localization_subscriber_creation(self):
        basic_navigator = BasicNavigator()
        self.assertIsNotNone(basic_navigator.localization_pose_sub)
        self.assertEqual(basic_navigator.localization_pose_sub.topic_name, '/amcl_pose')
        self.assertEqual(basic_navigator.localization_pose_sub.msg_type, PoseWithCovarianceStamped)
        basic_navigator.destroy_node() # Clean up the node

    def test_action_clients_creation(self):
        basic_navigator = BasicNavigator()
        self.assertIsNotNone(basic_navigator.nav_through_poses_client)
        self.assertEqual(basic_navigator.nav_through_poses_client.action_name, 'navigate_through_poses')
        self.assertEqual(basic_navigator.nav_through_poses_client.action_type, NavigateThroughPoses)

        self.assertIsNotNone(basic_navigator.nav_to_pose_client)
        self.assertEqual(basic_navigator.nav_to_pose_client.action_name, 'navigate_to_pose')
        self.assertEqual(basic_navigator.nav_to_pose_client.action_type, NavigateToPose)

        self.assertIsNotNone(basic_navigator.follow_waypoints_client)
        self.assertEqual(basic_navigator.follow_waypoints_client.action_name, 'follow_waypoints')
        self.assertEqual(basic_navigator.follow_waypoints_client.action_type, FollowWaypoints)

        self.assertIsNotNone(basic_navigator.compute_path_to_pose_client)
        self.assertEqual(basic_navigator.compute_path_to_pose_client.action_name, 'compute_path_to_pose')
        self.assertEqual(basic_navigator.compute_path_to_pose_client.action_type, ComputePathToPose)

        self.assertIsNotNone(basic_navigator.compute_path_through_poses_client)
        self.assertEqual(basic_navigator.compute_path_through_poses_client.action_name, 'compute_path_through_poses')
        self.assertEqual(basic_navigator.compute_path_through_poses_client.action_type, ComputePathThroughPoses)

        basic_navigator.destroy_node() # Clean up the node

    def test_service_clients_creation(self):
        basic_navigator = BasicNavigator()
        self.assertIsNotNone(basic_navigator.change_maps_srv)
        self.assertEqual(basic_navigator.change_maps_srv.srv_name, '/map_server/load_map')
        self.assertEqual(basic_navigator.change_maps_srv.srv_type, LoadMap)

        self.assertIsNotNone(basic_navigator.clear_costmap_global_srv)
        self.assertEqual(basic_navigator.clear_costmap_global_srv.srv_name, '/global_costmap/clear_entirely_global_costmap')
        self.assertEqual(basic_navigator.clear_costmap_global_srv.srv_type, ClearEntireCostmap)

        self.assertIsNotNone(basic_navigator.clear_costmap_local_srv)
        self.assertEqual(basic_navigator.clear_costmap_local_srv.srv_name, '/local_costmap/clear_entirely_local_costmap')
        self.assertEqual(basic_navigator.clear_costmap_local_srv.srv_type, ClearEntireCostmap)

        self.assertIsNotNone(basic_navigator.get_costmap_global_srv)
        self.assertEqual(basic_navigator.get_costmap_global_srv.srv_name, '/global_costmap/get_costmap')
        self.assertEqual(basic_navigator.get_costmap_global_srv.srv_type, GetCostmap)

        self.assertIsNotNone(basic_navigator.get_costmap_local_srv)
        self.assertEqual(basic_navigator.get_costmap_local_srv.srv_name, '/local_costmap/get_costmap')
        self.assertEqual(basic_navigator.get_costmap_local_srv.srv_type, GetCostmap)

        basic_navigator.destroy_node() # Clean up the node

if __name__ == '__main__':
    unittest.main()

# To run these tests:
# 1. Ensure you have a ROS 2 environment sourced.
# 2. Navigate to the root of this repository.
# 3. If you haven't built your package that includes the 'script' directory,
#    you might need to ensure Python can find 'robot_navigator'.
#    One way is to set PYTHONPATH:
#    export PYTHONPATH=$PYTHONPATH:$(pwd)/script
# 4. Run the tests using unittest:
#    python -m unittest test/test_robot_navigator.py
#
# Alternatively, if this were part of a ROS 2 package built with colcon,
# you would typically run:
# colcon test --packages-select <your_package_name>
# and then
# colcon test-result --verbose
