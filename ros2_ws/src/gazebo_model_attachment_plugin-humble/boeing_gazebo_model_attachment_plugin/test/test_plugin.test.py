#!/usr/bin/env python3

import os
import unittest
import rclpy
import launch
import launch.actions
import pytest  # type: ignore
import subprocess

import launch_testing
import launch_testing.actions
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from time import sleep

from boeing_gazebo_model_attachment_plugin.gazebo_client import GazeboModelAttachmentClient


@pytest.mark.launch_test
def generate_test_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world_file_name = os.path.join(get_package_share_directory('boeing_gazebo_model_attachment_plugin'),
                                   'test', 'test.world')
    print('world world_file_name : {}'.format(world_file_name))

    return launch.LaunchDescription(
        [
            # Launch GAZEBO
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch',
                                 'gzserver.launch.py')
                ),
                launch_arguments={
                    'world': world_file_name, 'gui': '0'}.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )

# There is a bug where the gzserver is not killed after the tests run.
# https://github.com/ros2/launch/issues/545
# The issue should be fixed by https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1376
# This is a workaround to kill the gzserver after the tests run.
# Remove this once gazebo updates the apt package to the latest version.


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_kill_sim(self):
        subprocess.run(["pkill", "gzserver"])
        subprocess.run(["pkill", "gzclient"])


class TestPlugin(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):

        self.__test_node = rclpy.create_node("test")

        self.__logger = self.__test_node.get_logger()

        sleep(5)  # Give time to Gazebo client/server to bring up

        self.__set_entity_state_srv = self.__test_node.create_client(
            srv_name='/gazebo/set_entity_state',
            srv_type=SetEntityState
        )

        self.__get_entity_state_srv = self.__test_node.create_client(
            srv_name='/gazebo/get_entity_state',
            srv_type=GetEntityState
        )

        self.__attachment_client = GazeboModelAttachmentClient()

    def test_attach(self):
        #
        # Test Model Attachment
        #

        self.__attachment_client.attach(
            joint_name='test_attachment_box',
            model_name_1='box',
            link_name_1='attachment_link',
            model_name_2='sphere',
            link_name_2='attachment_link'
        )

        self.__attachment_client.attach(
            joint_name='test_attachment_cylinder',
            model_name_1='box',
            link_name_1='attachment_link',
            model_name_2='cylinder',
            link_name_2='attachment_link'
        )

        self.__logger.info("Waiting for service: {}".format(self.__set_entity_state_srv.srv_name))
        if not self.__set_entity_state_srv.wait_for_service(timeout_sec=30):
            raise Exception("Timed out waiting for service: {}".format(self.__set_entity_state_srv.srv_name))

        # Ensure the sphere is correctly set as a child of the box
        future_set_entity = self.__set_entity_state_srv.call_async(
            SetEntityState.Request(
                state=EntityState(
                    name='box',
                    pose=Pose(
                        position=Point(x=0.0, y=0.0, z=2.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    ),
                    twist=Twist(
                        linear=Vector3(x=0.0, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=0.0)
                    ),
                    reference_frame=''
                )
            )
        )

        rclpy.spin_until_future_complete(self.__test_node, future_set_entity)
        self.assertIsInstance(future_set_entity.result(), SetEntityState.Response)

        self.__logger.info("Waiting for service: {}".format(self.__get_entity_state_srv.srv_name))
        if not self.__get_entity_state_srv.wait_for_service(timeout_sec=30):
            raise Exception("Timed out waiting for service: {}".format(self.__get_entity_state_srv.srv_name))

        future_box_entity_state = self.__get_entity_state_srv.call_async(
            GetEntityState.Request(
                name='box'
            )
        )
        rclpy.spin_until_future_complete(self.__test_node, future_box_entity_state)
        self.assertIsInstance(future_box_entity_state.result(), GetEntityState.Response)
        self.assertTrue(future_box_entity_state.result().success)

        self.__logger.info("Waiting for service: {}".format(self.__get_entity_state_srv.srv_name))
        if not self.__get_entity_state_srv.wait_for_service(timeout_sec=30):
            raise Exception("Timed out waiting for service: {}".format(self.__get_entity_state_srv.srv_name))

        future_sphere_entity_state = self.__get_entity_state_srv.call_async(
            GetEntityState.Request(
                name='sphere'
            )
        )
        rclpy.spin_until_future_complete(self.__test_node, future_sphere_entity_state)

        self.assertIsInstance(future_sphere_entity_state.result(), GetEntityState.Response)
        self.assertTrue(future_sphere_entity_state.result().success)

        self.assertEqual(future_sphere_entity_state.result().state.pose.position.y,
                         future_box_entity_state.result().state.pose.position.y)
        self.assertEqual(future_sphere_entity_state.result().state.pose.position.z,
                         future_box_entity_state.result().state.pose.position.z)
        self.assertEqual(future_sphere_entity_state.result().state.twist,
                         future_box_entity_state.result().state.twist)

        #
        # Test Model Detachment
        #

        self.__attachment_client.detach(
            joint_name='test_attachment_box',
            model_name_1='box',
            model_name_2='sphere',
        )

        # Ensure Cylinder isn't dropped as a child when the Sphere is

        self.__logger.info("Waiting for service: {}".format(self.__set_entity_state_srv.srv_name))
        if not self.__set_entity_state_srv.wait_for_service(timeout_sec=30):
            raise Exception("Timed out waiting for service: {}".format(self.__set_entity_state_srv.srv_name))

        response = self.__set_entity_state_srv.call_async(
            SetEntityState.Request(
                state=EntityState(
                    name='box',
                    pose=Pose(
                        position=Point(x=2.0, y=2.0, z=2.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    ),
                    twist=Twist(
                        linear=Vector3(x=0.0, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=0.0)
                    ),
                    reference_frame=''
                )
            )
        )
        rclpy.spin_until_future_complete(self.__test_node, response)

        self.assertIsInstance(response.result(), SetEntityState.Response)

        self.__logger.info("Waiting for service: {}".format(self.__get_entity_state_srv.srv_name))
        if not self.__get_entity_state_srv.wait_for_service(timeout_sec=30):
            raise Exception("Timed out waiting for service: {}".format(self.__get_entity_state_srv.srv_name))

        future_box_entity_state = self.__get_entity_state_srv.call_async(
            GetEntityState.Request(
                name='box'
            )
        )

        rclpy.spin_until_future_complete(self.__test_node, future_box_entity_state)
        self.assertIsInstance(future_box_entity_state.result(), GetEntityState.Response)
        self.assertTrue(future_box_entity_state.result().success)

        self.__logger.info("Waiting for service: {}".format(self.__get_entity_state_srv.srv_name))
        if not self.__get_entity_state_srv.wait_for_service(timeout_sec=30):
            raise Exception("Timed out waiting for service: {}".format(self.__get_entity_state_srv.srv_name))

        future_cylinder_entity_state = self.__get_entity_state_srv.call_async(
            GetEntityState.Request(
                name='cylinder'
            )
        )
        rclpy.spin_until_future_complete(self.__test_node, future_cylinder_entity_state)

        self.assertIsInstance(future_cylinder_entity_state.result(), GetEntityState.Response)
        self.assertTrue(future_cylinder_entity_state.result().success)

        self.assertEqual(future_cylinder_entity_state.result().state.pose.position.y,
                         future_box_entity_state.result().state.pose.position.y)
        self.assertEqual(future_cylinder_entity_state.result().state.pose.position.z,
                         future_box_entity_state.result().state.pose.position.z)
        self.assertEqual(future_cylinder_entity_state.result().state.twist,
                         future_box_entity_state.result().state.twist)
