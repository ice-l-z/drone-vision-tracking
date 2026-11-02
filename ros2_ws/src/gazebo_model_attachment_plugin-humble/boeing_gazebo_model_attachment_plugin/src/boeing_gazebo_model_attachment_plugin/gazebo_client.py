#!/usr/bin/env python3

import logging
import rclpy

from boeing_gazebo_model_attachment_plugin_msgs.srv import Attach
from boeing_gazebo_model_attachment_plugin_msgs.srv import Detach

logger = logging.getLogger(__name__)


class GazeboModelAttachmentClient(object):
    def __init__(self):
        self.__node = rclpy.create_node("gazebo_model_attachment_client")
        self.__logger = self.__node.get_logger()

        self.__attach_srv = self.__node.create_client(
            srv_name='/gazebo/attach',
            srv_type=Attach
        )

        self.__detach_srv = self.__node.create_client(
            srv_name='/gazebo/detach',
            srv_type=Detach
        )

    def attach(self, joint_name, model_name_1, link_name_1, model_name_2, link_name_2):
        # type: (str, str, str, str, str) -> None
        self.__logger.info("Waiting for service: {}".format(self.__attach_srv.srv_name))
        if not self.__attach_srv.wait_for_service(timeout_sec=30):
            raise Exception("Timed out waiting for service: {}".format(self.__attach_srv.srv_name))

        future = self.__attach_srv.call_async(
            Attach.Request(
                joint_name=joint_name,
                model_name_1=model_name_1,
                link_name_1=link_name_1,
                model_name_2=model_name_2,
                link_name_2=link_name_2
            )
        )  # type: Attach.Response
        rclpy.spin_until_future_complete(self.__node, future)

        assert isinstance(future.result(), Attach.Response)
        response = future.result()

        if response.success:
            self.__logger.info('Successfully attached models by adding joint {}'.format(joint_name))
            return response
        else:
            raise Exception('Failed to attach models: {}<--{}-->{} - {}'
                            .format(model_name_1, joint_name, model_name_2, response.message))

    def detach(self, joint_name, model_name_1, model_name_2):
        # type: (str, str, str) -> None
        self.__logger.info("Waiting for service: {}".format(self.__detach_srv.srv_name))
        if not self.__detach_srv.wait_for_service(timeout_sec=30):
            raise Exception("Timed out waiting for service: {}".format(self.__detach_srv.srv_name))

        future = self.__detach_srv.call_async(
            Detach.Request(
                joint_name=joint_name,
                model_name_1=model_name_1,
                model_name_2=model_name_2
            )
        )  # type: Detach.Response
        rclpy.spin_until_future_complete(self.__node, future)

        assert isinstance(future.result(), Detach.Response)
        response = future.result()

        if response.success:
            self.__logger.info('Successfully detached models by removing joint {}'.format(joint_name))
            return response
        else:
            raise Exception('Failed to detach models: {}<--{}-->{} - {}'
                            .format(model_name_1, joint_name, model_name_2, response.message))
