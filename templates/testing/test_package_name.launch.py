# Copyright (c) 2025 b»robotized
# All rights reserved.
#
# Proprietary License
#
# Unauthorized copying of this file, via any medium is strictly prohibited.
# The file is considered confidential
#
# Adapted for <Insert_Company_Name> that received unlimited, worldwide
# use and change right, except distributing this library separately
# of their product.

import unittest
import rclpy

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import launch_testing.actions

#TODO: add any other imports needed

def generate_test_description():

    minimal_urdf = """<?xml version="1.0"?><robot name="dummy_robot"><link name="base_link"/></robot>"""
    minimal_srdf = """<robot name="dummy_robot"><group name="dummy_group"><chain base_link="base_link" tip_link="base_link"/></group></robot>"""

    # TODO: Replace with whatever needed to be launched to run your system. You can include pre-existing launch files.
    # NODE_NAME = Node(
    #     package="PKG_NAME",
    #     executable="NODE_NAME",
    #     name="NODE_NAME",
    #     output="screen",
    # )

    # moveit_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[
    #         {"robot_description": minimal_urdf},
    #         {"robot_description_semantic": minimal_srdf},
    #         {"planning_pipelines": ["ompl"]},
    #         {"default_planning_pipeline": "ompl"},
    #         {"ompl": {"planning_plugins": ["ompl_interface/OMPLPlanner"]}},
    #         {"allow_trajectory_execution": False},
    #     ],
    # )

    # TODO: Change the time delay in TimerAction(period=0.5, ...) to whatever is needed to launch your system.
    return (
        LaunchDescription(
            [
                # moveit_node,
                # NODE_NAME,
                TimerAction(period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ),
        # {"moveit_launch": moveit_node, "custom_node": custom_node},
    )


# Active tests
class TestPkgName(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS 2 node for testing
        self.node = rclpy.create_node("test_custom_node")

        # TODO: Create a client for each service or action
        # self.my_service_client = self.node.create_client(MyServiceType, "my_service")

    def tearDown(self):
        self.node.destroy_node()

    # TODO: Edit the testcase as required. Each testcase should start by triggering a behavior 
    # (i.e. calling a service or subscribing to a topic) and end by asserting the response.
    # def test_my_custom_service(self):
    #     # Wait for the service to be available
    #     self.assertTrue(
    #         self.my_service_client.wait_for_service(timeout_sec=5.0),
    #         "my_service service not available",
    #     )

	# 	request = MyServiceType.Request()

    #     # Call the service
    #     future = self.my_service_client.call_async(request)
    #     rclpy.spin_until_future_complete(self.node, future)

    #     # Check the response
    #     response = future.result()
    #     self.assertIsNotNone(response, "No response received from my_service service")
    #     self.assertEqual(response.result, 1, "my_service service should succeed!")
    
    # TODO: After you brainstormed your testcases, name and add them to the class in separate functions. Each function is a testcase.