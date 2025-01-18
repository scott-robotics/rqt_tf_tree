# Software License Agreement (BSD License)
#
# Copyright (c) 2025, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import yaml
import rclpy

from tf2_msgs.srv import FrameGraph
from geometry_msgs.msg import TransformStamped

import tf_transformations as tft
import tf2_ros


class RosTfHandler:
    def __init__(self, node: rclpy.node.Node):
        self._node = node

        self._tf2_buffer = tf2_ros.Buffer(node=self._node)
        self._tf2_listener = tf2_ros.TransformListener(self._tf2_buffer, self._node)
        self._graph_client = self._node.create_client(FrameGraph, "tf2_frames")

        self._format_str = "\n  xyz: {}\n  rpy: {}"

    def lookup_transform_as_string(
        self,
        target_frame: str,
        source_frame: str,
        timestamp: float,
        timeout: float = 0.0,
    ) -> str:
        try:
            tf = self._tf2_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(seconds=timestamp),
                timeout=rclpy.duration.Duration(seconds=timeout))
        except Exception as e:
            return str(e)

        quaternion = tf.transform.rotation
        qw, qx, qy, qz = quaternion.w, quaternion.x, quaternion.y, quaternion.z

        translation = tf.transform.translation
        xyz = [translation.x, translation.y, translation.z]
        rpy = tft.euler_from_quaternion([qx, qy, qz, qw])

        xyz_f = map("{:.3f}".format, xyz)
        rpy_f = map("{:.3f}".format, rpy)

        return self._format_str.format(", ".join(xyz_f), ", ".join(rpy_f))

    def get_graph(self, yaml_parser=yaml) -> dict:
        yaml_data = self._graph_client.call(FrameGraph.Request()).frame_yaml
        return yaml_parser.safe_load(yaml_data)

    def clear_buffers(self):
        self._tf2_buffer.clear()

    def wait_for_service(self):
        while not self._graph_client.wait_for_service(timeout_sec=1.0):
            print("service not available, waiting again...")
