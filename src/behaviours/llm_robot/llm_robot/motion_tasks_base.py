#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2025 Howe @
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
#
# Description:
# This example demonstrates simulating function calls for any robot,



# ROS related
from functools import partial
from inspect import signature
import json
import time
import rclpy
from copy import deepcopy
from rclpy.node import Node
from rclpy.action import ActionClient
from llm_interfaces.srv import ChatLLM, MoveJoint,RecognizePickPlace,MoveCartesian,RelativeMove
from std_msgs.msg import String,  Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from std_srvs.srv import Empty
from geometry_msgs.msg import Point
from llm_interfaces.action import ArmMoveTo,MoveToPickPlace
from aubo_msgs.srv import GetSignalStatus,SetOutputSignal
from action_msgs.msg import GoalStatus
from rclpy.executors import MultiThreadedExecutor
#BT tree releated
import py_trees
import py_trees_ros
from py_trees.trees import BehaviourTree
from py_trees.behaviours import Success, Running, Failure,SuccessEveryN
from py_trees.common import Status
from py_trees.decorators import FailureIsSuccess
import py_trees.blackboard as bb

import threading

from concurrent.futures import ThreadPoolExecutor
import asyncio

class RecognizePickPlaceNode(py_trees.behaviour.Behaviour):
    def __init__(self,node, name):
        super(RecognizePickPlaceNode, self).__init__(name)
        self.node = node
        
        self.bb = py_trees.blackboard.Blackboard()
        self.bb.set("pick_items", None)
        self.bb.set("place_item", None)
        self.bb.set("base2cam_transform", None)
        self.status = py_trees.common.Status.INVALID  # 初始化状态为 INVALID
    def initialise(self):
        while not self.node._recognize_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.node.get_logger().error('Interruped while waiting for the server.')
                return
            else:
                self.node.get_logger().info('Recognize Server not available, waiting again...')
                self.status = py_trees.common.Status.INVALID
                break
    def update(self):
        self.pick_object_type = self.bb.get("pick_objects")
        self.place_object_type = self.bb.get("place_object")
        # self.node.get_logger().info(f'update with pick_type: {self.pick_object_type} and place_type: {self.place_object_type}')
        # self.node.get_logger().info(f"RecognizePickPlaceNode-update: {self.node.pick_objects} and {self.node.place_object}")
        if self.pick_object_type is None and self.place_object_type is None:
            return py_trees.common.Status.FAILURE
        recognize_request = RecognizePickPlace.Request()
        recognize_request.pick_object_type = self.pick_object_type
        recognize_request.place_object_type = self.place_object_type
        response = self.node._recognize_client.call_async(recognize_request)
        rclpy.spin_until_future_complete(self.node,response)
        if response.result() is not None:
            if response.result().pick_items and response.result().place_item:
                self.node.get_logger().info(f'Recognize succeeded with pick_items: {response.result().pick_items} and place_item: {response.result().place_item}')
                self.bb.set("pick_items", response.result().pick_items)
                self.bb.set("place_item", response.result().place_item)
                self.bb.set("base2cam_transform",response.result().base2cam_transform)
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.FAILURE

class MoveJointBehavior(py_trees.behaviour.Behaviour):

    def __init__(self,node,name="MoveJoint", target_joints=None):
        super(MoveJointBehavior, self).__init__(name)
        self.node = node
        self.target_joints = target_joints
        self.bb = py_trees.blackboard.Blackboard()
        self._move_joint_client = self.node.create_client(MoveJoint,"/motion_control/move_joint")
    def initialise(self):
        while not self._move_joint_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.node.get_logger().error('Interruped while waiting for the server.')
                return
            else:
                self.node.get_logger().info('MoveJoint Server not available, waiting again...')
    def update(self):
        movejoint_request = MoveJoint.Request()
        movejoint_request.angles = self.target_joints
        response = self._move_joint_client.call_async(movejoint_request)
        # response = self._move_joint_client.call(movejoint_request)
        self.node.get_logger().info(f"move_joint call: {self.target_joints}")
        rclpy.spin_until_future_complete(self.node,response)
        if response.result() is not None and response.result().success:
            self.node.get_logger().info("movejoint_response: success")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info("movejoint_response: FAILURE")
            return py_trees.common.Status.FAILURE

class MoveByObjectFramePointBehavior(py_trees.behaviour.Behaviour):

    def __init__(self,node,name="MoveByObjectFramePointBehavior",base2cam_transform=None, target_position=None,tcp='tool0',use_entry_point=False,entry_height=0.1):
        super(MoveByObjectFramePointBehavior, self).__init__(name)
        self.node = node
        self.node.get_logger().info(f'run MoveByObjectFramePointBehavior name:{name} target_posiont: {target_position}')
        self.base2cam_transform = base2cam_transform
        self.target_position = target_position if target_position else Point()
        self.entry_height = entry_height
        self.use_entry_point = use_entry_point
        self.pose_link = tcp

    def initialise(self):
        self.node.get_logger().info(f'run MoveByObjectFramePointBehavior initialise')
        while not self.node._move_cartesian_object_frame_point_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.node.get_logger().error('Interruped while waiting for the server.')
                return
            else:
                self.node.get_logger().info('Server not available, waiting again...')
        self.status = py_trees.common.Status.INVALID

    def update(self):
        self.node.get_logger().info(f'run MoveByObjectFramePointBehavior update')
        request = MoveCartesian.Request()
        if self.base2cam_transform is not None:
            request.base2cam_transform = self.base2cam_transform
        request.object_point = self.target_position
        request.object_frame = 'camera_color_optical_frame'
        request.use_entry_point = self.use_entry_point
        request.entry_point_height = self.entry_height
        request.pose_link = self.pose_link
        response = self.node._move_cartesian_object_frame_point_client.call_async(request)
        rclpy.spin_until_future_complete(self.node,response)
        if response.result() is not None and response.result().success:
            self.node.get_logger().info("MoveByObjectFramePointBehavior response success")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class MoveByRelativeBehavior(py_trees.behaviour.Behaviour):

    def __init__(self,node,name="MoveByRelativeBehavior", relative_pose=[0.0,0.0,0.0,0.0,0.0,0.0],reference_frame=0,tcp='tool0'):
        super(MoveByRelativeBehavior, self).__init__(name)
        self.node = node
        self.relative_pose = relative_pose
        self.reference_frame = reference_frame
        self.pose_link = tcp

    def initialise(self):
        while not self.node._move_relative_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.node.get_logger().error('Interruped while waiting for the server.')
                return
            else:
                self.node.get_logger().info('Server not available, waiting again...')

    def update(self):
        request = RelativeMove.Request()
        request.dx = self.relative_pose[0]
        request.dy = self.relative_pose[1]
        request.dz = self.relative_pose[2]
        request.drx = self.relative_pose[3]
        request.dry = self.relative_pose[4]
        request.drz = self.relative_pose[5]
        request.reference_frame = self.reference_frame
        request.pose_link = self.pose_link
        response = self.node._move_relative_client.call_async(request)
        rclpy.spin_until_future_complete(self.node,response)
        if response.result() is not None and response.result().success:
            self.node.get_logger().info("MoveByRelativeBehavior response success")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class GripperControl(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="GripperControl",index=0,value=0):
        super(GripperControl, self).__init__(name)
        self.node = node
        self.signal_index = index
        self.signal_value = value

    def update(self):
        signal_request = SetOutputSignal.Request()
        signal_request.signal_name_index = self.signal_index
        signal_request.value = self.signal_value
        response = self.node.signal_output.call_async(signal_request)
        rclpy.spin_until_future_complete(self.node,response)
        if response.result() is not None:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class GripperGetStatus(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="GripperGetStatus",index=0):
        super(GripperGetStatus, self).__init__(name)
        self.node = node
        self.signal_index = index
        self.bb = bb.Blackboard()
        self.bb.set("Gripper_status", None)

    def update(self):
        signal_request = GetSignalStatus.Request()
        signal_request.signal_name_index = self.signal_index
        response = self.node.signal_get.call_async(signal_request)
        rclpy.spin_until_future_complete(self.node,response)
        if response.result() is not None:
            self.bb.set("Gripper_status", response.result().status)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class CheckAndOpenGripper(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="CheckAndOpenGripper", index=0, open_value=1000):
        super(CheckAndOpenGripper, self).__init__(name)
        self.node = node
        self.index = index
        self.open_value = open_value
        self.bb = py_trees.blackboard.Blackboard()
        self.Gripper_get_status = GripperGetStatus(node, index=1)
        self.Gripper_control_open = GripperControl(node, index=index, value=open_value)

    def update(self):
        self.node.get_logger().info("CheckAndOpenGripper-update")
        
        # Step 1: 获取当前夹爪状态
        status_status = self.Gripper_get_status.update()
        if status_status != py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.FAILURE

        # 获取夹爪状态值
        Gripper_status = self.Gripper_get_status.bb.get("Gripper_status")

        # Step 2: 如果状态值小于 900，打开夹爪
        if Gripper_status < 900:
            open_status = self.Gripper_control_open.update()
            if open_status != py_trees.common.Status.SUCCESS:
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

class AttachOrDetach_Up(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="AttachOrDetach_Up", attach_or_detach = 0, index=0):
        super(AttachOrDetach_Up, self).__init__(name)
        self.node = node
        self.index = index
        self.attach_or_detach = attach_or_detach
        self.Gripper_control = GripperControl(node, index=index, value=100 if self.attach_or_detach ==0 else 1000)  # 关闭夹爪
        self.Gripper_get_status = GripperGetStatus(node, index=1)
        increase_up = [0.0,0.0,0.1,0.0,0.0,0.0]
        self.move_to_up = MoveByRelativeBehavior(self.node,name="Move Up",relative_pose=increase_up)

    def update(self):
        # Step 1: 关闭夹爪
        control_status = self.Gripper_control.update()
        if control_status != py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.FAILURE
        # # Step 2: 获取夹爪状态
        # status_status = self.Gripper_get_status.update()
        # if status_status != py_trees.common.Status.SUCCESS:
        #     return py_trees.common.Status.FAILURE
        # # 获取抓取状态值
        # Gripper_status = self.Gripper_get_status.bb.get("Gripper_status")
        # # 判断夹爪状态，<200 或 >900 返回FAILURE
        # if Gripper_status < 200 or Gripper_status > 900:
        #     return py_trees.common.Status.FAILURE
        # else:
            # Step 3: 抓起
        if self.attach_or_detach ==0:
            time.sleep(2)
        else:
            time.sleep(0.5)
        move_to_up_status = self.move_to_up.update()
        self.node.get_logger().info(f"AttachOrDetach_Up update status:{move_to_up_status}")
        if move_to_up_status != py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS

class PickAndPlaceSequence(py_trees.behaviour.Behaviour):
    def __init__(self,node,name="PickAndPlaceSequence"):
        super(PickAndPlaceSequence, self).__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Blackboard()
        self.sequence = None
    def initialise(self):
        pick_items = self.bb.get("pick_items")
        place_item = self.bb.get("place_item")
        base2cam_transform = self.bb.get("base2cam_transform")
        if pick_items and place_item:
            self.node.get_logger().info(f"Post-tick place_item: {place_item}")
            self.sequence = py_trees.composites.Sequence(name="DynamicSequence",memory=False)
            
            for item_index in range(len(pick_items)):
                pick_item_key = f"pick_item_{item_index}"
                pick_item = pick_items[item_index]
                self.bb.set(pick_item_key,pick_item)
                place_item_up = deepcopy(place_item)
                place_item_up.z -= 0.05 * (item_index + 1)
                move_to_capture = MoveJointBehavior(self.node,name="Move to Capture", target_joints=self.bb.get("capture_point"))
                move_to_place = MoveByObjectFramePointBehavior(self.node,"MoveToPlace",base2cam_transform, target_position=place_item,entry_height=0.1)
                move_to_pick = MoveByObjectFramePointBehavior(self.node,f"MoveToPick_{item_index}", base2cam_transform, target_position=pick_item,entry_height=0.1)
                pickup = AttachOrDetach_Up(self.node,name=f"Pickup_{item_index}")
                move_to_place_up = MoveByObjectFramePointBehavior(self.node,"MoveToPlace", base2cam_transform,target_position=place_item_up,entry_height=0.1)
                placeup = AttachOrDetach_Up(self.node,name="PlaceItem_Up",attach_or_detach=1)
                if base2cam_transform:
                    self.sequence.add_children([move_to_pick,pickup,move_to_place_up,placeup])
                else:
                    self.sequence.add_children([move_to_capture,move_to_pick,pickup,move_to_capture,move_to_place_up,placeup])
        else:
            self.sequence = Failure(name="NoTasks")
        # 初始化子行为树
        self.sequence.setup_with_descendants()

    def update(self):
        if self.sequence is not None:
            self.node.get_logger().info("PickAndPlaceSequence: update")
            # Tick子行为树，并检查是否完成
            self.sequence.tick_once()
            status = self.sequence.status
            self.node.get_logger().info(f"PickAndPlaceSequence: update status: {status}")
            return status
        else:
            self.node.get_logger().info("PickAndPlaceSequence none: update failure")
            return py_trees.common.Status.FAILURE
            
class MotionRobot(Node):
    def __init__(self):
        super().__init__("MotionRobot_node")
        #Service Client
        self._recognize_client = self.create_client(RecognizePickPlace,"vlm/recognize_pick_place_item")
        self._move_joint_client = self.create_client(MoveJoint,"/motion_control/move_joint")
        self._move_cartesian_client = self.create_client(MoveCartesian,'motion_control/move_cartesian')
        self._move_cartesian_euler_client = self.create_client(MoveCartesian,'motion_control/move_cartesian_euler')
        self._move_cartesian_quat_client = self.create_client(MoveCartesian,'motion_control/move_cartesian_quat')
        self._move_cartesian_object_frame_point_client = self.create_client(MoveCartesian,'motion_control/move_cartesian_objetc_frame_point')
        self._move_relative_client = self.create_client(RelativeMove,'motion_control/realtive_move')

        self.signal_output = self.create_client(SetOutputSignal, "/io_and_status_controller/modbus_set_output_signal")
        self.signal_get = self.create_client(GetSignalStatus, "/io_and_status_controller/modbus_get_signal_status")
        self.signal_output.wait_for_service(timeout_sec=3.0)
        self.vlm_feedback_publisher = self.create_publisher(
            String, "/vlm_feedback_to_user", 0
        )
        # Server for function call
        self.function_call_server = self.create_service(
            ChatLLM, "/MotionRobot_function_call_service", self.function_call_callback
        )
        # Node initialization log
        self.bb = bb.Blackboard()
        self.bb.set("pick_objects", None)
        self.bb.set("place_object", None)
        self.get_logger().info("MotionRobot node has been initialized")
        self.data_received = False
        self.tree = self.create_behavior_tree()
        # self.get_logger().info("Behavior tree created")

    def function_call_callback(self, request, response):
        reqs = json.loads(request.request_text)
        rets = []
        for req in reqs:
            function_name = req["name"]
            function_args = req["params"]
            func_obj = getattr(self, function_name)
            if func_obj is None:
                rets.append(f"Function '{function_name}' not found.")
                continue
            try:
                # 检查函数签名是否与参数匹配
                sig = signature(func_obj)
                bound_args = sig.bind(**function_args)
                bound_args.apply_defaults()
                # 使用 functools.partial 构造函数调用
                function_execution_result = partial(func_obj, **bound_args.arguments)()
            except Exception as error:
                print(f"Failed to call function: {error}")
                ret = str(error)
            else:
                ret = str(function_execution_result)
            rets.append(ret)
        response.response_text = str(rets)
        return response

    def create_behavior_tree(self):
        root = py_trees.composites.Sequence(name="LLM BT", memory=False)
        blackboard = bb.Blackboard()
        move_to_capture = MoveJointBehavior(self,name="Move to Capture", target_joints=blackboard.get("capture_point"))
        move_to_home = MoveJointBehavior(self,name="Move to Home", target_joints=blackboard.get("home_position"))
        recognize = RecognizePickPlaceNode(self,name="Recognize Objects")
        check_and_open_gripper = CheckAndOpenGripper(self,name="check_gripper",index=0,open_value=1000)
        pick_place_sequence = PickAndPlaceSequence(self)
        # test_point = Point(x=-0.1762021844460045, y=0.005601765347258108, z=0.356)
        # move_to_pick = MoveByObjectFramePointBehavior(self, target_position=test_point,use_entry_point=True,entry_height=0.1)
        # increase_up = [0.0,0.0,0.1,0.0,0.0,0.0]
        # move_to_up = MoveByRelativeBehavior(self,name="Move Up",relative_pose=increase_up)
        # root.add_children([move_to_capture,recognize,pick_place_sequence])
        # root.add_children([move_to_capture])
        root.add_children([move_to_capture,recognize,check_and_open_gripper,pick_place_sequence])
         # Create the initial behavior tree and tick it
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(timeout=15)  # Initialize the tree
        return tree
    def execute_behavior_tree(self):
        while rclpy.ok():
            if self.data_received:  # 判断标志变量
                self.tree.tick()
                print(py_trees.display.unicode_tree(root=self.tree.root, show_status=True))
                self.data_received = False  # 重置标志变量
            rclpy.spin_once(self, timeout_sec=5.0)
            time.sleep(2)
def main():
    rclpy.init()
    node = MotionRobot()
    node.execute_behavior_tree()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
     main()
