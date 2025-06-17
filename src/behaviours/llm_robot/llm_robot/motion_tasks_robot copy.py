#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2024 ZhangDuo @AUBO Robotics
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
# such as controlling velocity and other service commands.
# By modifying the content of this file,
# A calling interface can be created for the function calls of any robot.
# The Python script creates a ROS 2 Node
# that controls the movement of the TurtleSim
# by creating a publisher for cmd_vel messages and a client for the reset service.
# It also includes a ChatLLM function call server
# that can call various functions to control the TurtleSim
# and return the result of the function call as a string.


# ROS related
from functools import partial
from inspect import signature
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from llm_interfaces.srv import ChatLLM, MoveJoint,RecognizePickPlace
from std_msgs.msg import String,  Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from std_srvs.srv import Empty
from geometry_msgs.msg import Point
from llm_interfaces.action import ArmMoveTo,MoveToPickPlace
from aubo_msgs.srv import GetSignalStatus,SetOutputSignal
from action_msgs.msg import GoalStatus
#BT tree releated
import py_trees
from py_trees.trees import BehaviourTree
import py_trees.blackboard as bb
import py_trees_ros
import threading
from py_trees.common import Status

# class RecognizePickPlaceNode(py_trees.behaviour.Behaviour):
#     def __init__(self,node, name, pick_object_type, place_object_type):
#         super(RecognizePickPlaceNode, self).__init__(name)
#         self.node = node
#         self.action_client = ActionClient(self.node, RecognizePickPlace, 'recognize_objects')
#         self.pick_object_type = pick_object_type
#         self.place_object_type = place_object_type
#         self.blackboard = py_trees.blackboard.Blackboard()
#         self.blackboard.set("pick_items", None)
#         self.blackboard.set("place_item", None)
#         self.goal_handle = None
#         self.result = None
#         self._is_done = False  # To track if the goal has completed
#         self.status = py_trees.common.Status.INVALID    # Track the status internally
#     def initialise(self):
#         """Called when the behavior tree node is initialized."""
#         self.node.get_logger().info(f'Initializing with pick_object_type: {self.pick_object_type}, place_object_type: {self.place_object_type}')

#     def update(self):
        
#         if not self._is_done:
#             if self.goal_handle is None:  # Only send the goal if it hasn't been sent yet
#                 goal_msg = RecognizePickPlace.Goal()
#                 goal_msg.pick_object_type = self.pick_object_type
#                 goal_msg.place_object_type = self.place_object_type
#                 self.action_client.wait_for_server()
#                 self._send_goal_future = self.action_client.send_goal_async(
#                     goal_msg, feedback_callback=self.feedback_callback)
#                 self._send_goal_future.add_done_callback(self.goal_response_callback)
#                 self.node.get_logger().info('Goal sent to recognize objects')
#                 return py_trees.common.Status.RUNNING
#         # If the goal is done, determine if it succeeded or failed
#         if self.result is not None:
#             if self.result.pick_items and self.result.place_item:
#                 self.node.get_logger().info(f'Recognize succeeded with pick_items: {self.result.pick_items} and place_item: {self.result.place_item}')
#                 self.blackboard.set("pick_items", self.result.pick_items)
#                 self.blackboard.set("place_item", self.result.place_item)
#                 self.status = py_trees.common.Status.SUCCESS
#             else:
#                 self.status = py_trees.common.Status.FAILURE
#         else:
#             self.status = py_trees.common.Status.FAILURE
#         self.node.get_logger().info(f'RecognizePickPlaceNode update {self.status}')
#         return self.status

#     def goal_response_callback(self, future):
#         self.goal_handle = future.result()
#         if not self.goal_handle.accepted:
#             self.node.get_logger().info('Goal rejected')
#             self.status = py_trees.common.Status.FAILURE
#             self._is_done = True
#         else:
#             self.node.get_logger().info('Goal accepted')
#             self._get_result_future = self.goal_handle.get_result_async()
#             self._get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         self.result = future.result().result
#         self._is_done = True  # Indicate that the goal has completed
#         # if self.result.pick_items and self.result.place_item:
#         #     self.node.get_logger().info(f'Recognize succeeded with pick_items: {self.result.pick_items} and place_item: {self.result.place_item}')
#         #     self.blackboard.set("pick_items", self.result.pick_items)
#         #     self.blackboard.set("place_item", self.result.place_item)
#         #     return py_trees.common.Status.SUCCESS
#         # else:
#         #     self.node.get_logger().info('Recognize failed, no items detected')
#         #     return py_trees.common.Status.FAILURE

#     def feedback_callback(self, feedback_msg):
#         self.node.get_logger().info(f'Feedback: {feedback_msg.feedback.current_status}')

#     def terminate(self, new_status):
#         self.node.get_logger().info('terminate')
#         if self.goal_handle is not None:
#             self.goal_handle.cancel_goal_async()
#         self.goal_handle = None
#         self.result = None
#         self._is_done = False

# class RecognizePickPlaceNode2(py_trees.behaviour.Behaviour):
#     def __init__(self,node, name, pick_object_type, place_object_type):
#         super(RecognizePickPlaceNode2, self).__init__(name)
#         self.node = node
#         self._action_client = ActionClient(self.node, RecognizePickPlace, 'recognize_objects')
#         self.pick_object_type = pick_object_type
#         self.place_object_type = place_object_type
#         self.blackboard = py_trees.blackboard.Blackboard()
#         self.blackboard.set("pick_items", None)
#         self.blackboard.set("place_item", None)
#         self._status_thread = None
#         self._status_lock = threading.Lock()
#         self._status = None

#     def send_goal(self):
#         goal_msg = RecognizePickPlace.Goal()
#         goal_msg.pick_object_type = self.pick_object_type
#         goal_msg.place_object_type = self.place_object_type
#         self._action_client.wait_for_server(timeout_sec=2.0)
#         time.sleep(3)
#         self._send_goal_future = self._action_client.send_goal_async(goal_msg)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)
#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             with self._status_lock:
#                 self._status = py_trees.common.Status.FAILURE
#             return
#         self._status_thread = threading.Thread(target=self.monitor_goal_status, args=(goal_handle,))
#         self._status_thread.start()
#     def monitor_goal_status(self, goal_handle):
#         # Monitor the goal status and update accordingly
#         result_future = goal_handle.get_result_async()
#         result = None
#         while result is None:
#             if result_future.done():
#                 result = result_future.result()
#             # else:
#             #     rclpy.spin_once(self.node, timeout_sec=1.0)  # wait for result or spin to process callbacks

#         with self._status_lock:
#             # self.node.get_logger().info(result)
#             """llm_interfaces.action.RecognizePickPlace_GetResult_Response
#             (status=4, result=llm_interfaces.action.RecognizePickPlace_Result
#             (pick_items=[geometry_msgs.msg.Point(x=-0.215751546330228, y=-0.005354300927629775, z=0.392)],
#             place_item=geometry_msgs.msg.Point(x=-0.06741699135989442, y=0.017607210235070853, z=0.391)))'"""
#             if result is not None and result.result is not None:
#                 if result.result.pick_items and result.result.place_item:
#                     self.node.get_logger().info(f'Recognize succeeded with pick_items: {result.result.pick_items} and place_item: {result.result.place_item}')
#                     self.blackboard.set("pick_items", result.result.pick_items)
#                     self.blackboard.set("place_item", result.result.place_item)
#                     self._status = py_trees.common.Status.SUCCESS
#                 else:
#                     self._status = py_trees.common.Status.FAILURE
#             else:
#                 self._status = py_trees.common.Status.FAILURE

#     def update(self):
#         self.node.get_logger().info(f'---update status:{self._status}')
#         if self._status is None:
#             self.send_goal()
#             return py_trees.common.Status.RUNNING
#         with self._status_lock:
#             return self._status

#     def feedback_callback(self, feedback_msg):
#         self.node.get_logger().info(f'Feedback: {feedback_msg.feedback.current_status}')

class RecognizePickPlaceNode(py_trees.behaviour.Behaviour):
    def __init__(self,node, name, pick_object_type, place_object_type):
        super(RecognizePickPlaceNode, self).__init__(name)
        self.node = node
        self.pick_object_type = pick_object_type
        self.place_object_type = place_object_type
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.set("pick_items", None)
        self.blackboard.set("place_item", None)
        self.goal_handle = None
        self.result = None
        self._is_done = False  # To track if the goal has completed
        self.status = py_trees.common.Status.INVALID    # Track the status internally
    def initialise(self):
        """Called when the behavior tree node is initialized."""
        self.node.get_logger().info(f'Initializing with pick_object_type: {self.pick_object_type}, place_object_type: {self.place_object_type}')
        while not self.node._recognize_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.node.get_logger().error('Interruped while waiting for the server.')
                return
            else:
                self.node.get_logger().info('Server not available, waiting again...')
        self.status = py_trees.common.Status.RUNNING
    def update(self):
        recognize_request = RecognizePickPlace.Request()
        recognize_request.pick_object_type = self.pick_object_type
        recognize_request.place_object_type = self.place_object_type
        response = self.node._recognize_client.call_async(recognize_request)
        self.node.get_logger().info(f"_recognize_client call: {self.pick_object_type} {self.place_object_type}")
        rclpy.spin_until_future_complete(self.node,response)
        if response.result() is not None:
            if response.result().pick_items and response.result().place_item:
                self.node.get_logger().info(f'Recognize succeeded with pick_items: {response.result().pick_items} and place_item: {response.result().place_item}')
                self.blackboard.set("pick_items", response.result().pick_items)
                self.blackboard.set("place_item", response.result().place_item)
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
        self.blackboard = py_trees.blackboard.Blackboard()
    def initialise(self):
        while not self.node._move_joint_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.node.get_logger().error('Interruped while waiting for the server.')
                return
            else:
                self.node.get_logger().info('Server not available, waiting again...')
        self.status = py_trees.common.Status.RUNNING
    def update(self):
        self.node.get_logger().info("MoveJointBehavior-update")
        pick_items = self.blackboard.get("pick_items")
        place_item = self.blackboard.get("place_item")
        
        if pick_items and place_item:
            self.node.get_logger().info(f"MJ Post-tick pick_items: {pick_items}")
            self.node.get_logger().info(f"MJ Post-tick place_item: {place_item}")
        movejoint_request = MoveJoint.Request()
        movejoint_request.angles = self.target_joints
        response = self.node._move_joint_client.call_async(movejoint_request)
        self.node.get_logger().info(f"move_joint call: {self.target_joints}")
        rclpy.spin_until_future_complete(self.node,response)
        if response.result() is not None:
            self.node.get_logger().info(f"movejoint_response: {response.result()}")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class MoveToPickPlaceBehavior(py_trees.behaviour.Behaviour):

    def __init__(self,node,name="MoveTo", target_position=None,updown=0.1):
        super(MoveToPickPlaceBehavior, self).__init__(name)
        self.node = node
        self.target_position = target_position if target_position else Point()
        self.updown = updown
        self.action_client = ActionClient(self.node, MoveToPickPlace, 'move_to')
        self.goal_handle = None
        self.result = None

    def setup(self, timeout):
        self.action_client.wait_for_server()
        return True

    def initialise(self):
        goal_msg = MoveToPickPlace.Goal()
        goal_msg.object_position = self.target_position
        goal_msg.updown_height = self.updown
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def update(self):
        if self.goal_handle is None:
            return py_trees.common.Status.RUNNING

        if self.result is not None:
            if self.result.success: # type: ignore
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.result = False
            self.node.get_logger().info('Goal rejected')

    def get_result_callback(self, future):
        self.result = future.result().result
        self.node.get_logger().info(f'Result: {"Success" if self.result.success else "Failure"}')

    def feedback_callback(self, feedback_msg):
        self.node.get_logger().info(f'Feedback: {feedback_msg.feedback.current_status}')

    def terminate(self, new_status):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

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
        self.blackboard = bb.Blackboard()
        self.blackboard.set("Gripper_status", None)

    def update(self):
        signal_request = GetSignalStatus.Request()
        signal_request.signal_name_index = self.signal_index
        response = self.node.signal_get.call_async(signal_request)
        rclpy.spin_until_future_complete(self.node,response)
        if response.result() is not None:
            self.blackboard.set("Gripper_status", response.result().status)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class CheckAndOpenGripper(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="CheckAndOpenGripper", index=0, open_value=1000):
        super(CheckAndOpenGripper, self).__init__(name)
        self.node = node
        self.index = index
        self.open_value = open_value
        self.blackboard = py_trees.blackboard.Blackboard()
        self.Gripper_get_status = GripperGetStatus(node, index=1)
        self.Gripper_control_open = GripperControl(node, index=index, value=open_value)

    def update(self):
        self.node.get_logger().info("CheckAndOpenGripper-update")
        
        # Step 1: 获取当前夹爪状态
        status_status = self.Gripper_get_status.update()
        if status_status != py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.FAILURE

        # 获取夹爪状态值
        Gripper_status = self.Gripper_get_status.blackboard.get("Gripper_status")

        # Step 2: 如果状态值小于 900，打开夹爪
        if Gripper_status < 900:
            open_status = self.Gripper_control_open.update()
            if open_status != py_trees.common.Status.SUCCESS:
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

class PickObject(py_trees.behaviour.Behaviour):
    def __init__(self, node, object_position_key,name="PickObject", index=0):
        super(PickObject, self).__init__(name)
        self.node = node
        self.index = index
        self.blackboard = bb.Blackboard()
        self.object_position = self.blackboard.get(object_position_key)
        self.object_position.z +=0.1
        self.Gripper_control = GripperControl(node, index=index, value=100)  # 关闭夹爪
        self.Gripper_get_status = GripperGetStatus(node, index=2)
        self.move_to_up = MoveJointBehavior(self,name="Move to Target", target_position=self.object_position)

    def update(self):
        # Step 1: 关闭夹爪
        control_status = self.Gripper_control.update()
        if control_status != py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.FAILURE
        # Step 2: 获取夹爪状态
        status_status = self.Gripper_get_status.update()
        if status_status != py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.FAILURE
        # 获取抓取状态值
        Gripper_status = self.Gripper_get_status.blackboard.get("Gripper_status")
        # 判断夹爪状态，<200 或 >900 返回FAILURE
        if Gripper_status < 200 or Gripper_status > 900:
            return py_trees.common.Status.FAILURE
        else:
            # Step 3: 抓起
            move_to_up_status = self.move_to_up.update()
            if move_to_up_status != py_trees.common.Status.SUCCESS:
                return py_trees.common.Status.FAILURE
            else:
                return py_trees.common.Status.SUCCESS

class PlaceObject(py_trees.behaviour.Behaviour):
    def __init__(self, node,name="PlaceObject", index=0):
        super(PlaceObject, self).__init__(name)
        self.node = node
        self.index = index
        self.blackboard = bb.Blackboard()
        self.object_position = self.blackboard.get("place_item")
        self.object_position.z +=0.1
        self.Gripper_control = GripperControl(node, index=index, value=1000)  # 打开夹爪
        self.Gripper_get_status = GripperGetStatus(node, index=2)
        self.move_to_up = MoveJointBehavior(self,name="Move to Target", target_position=self.object_position)

    def update(self):
        # Step 1: 打开夹爪
        control_status = self.Gripper_control.update()
        if control_status != py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.FAILURE
        # Step 2: 获取夹爪状态
        status_status = self.Gripper_get_status.update()
        if status_status != py_trees.common.Status.SUCCESS:
            return py_trees.common.Status.FAILURE
        # 获取抓取状态值
        Gripper_status = self.Gripper_get_status.blackboard.get("Gripper_status")
        # 判断夹爪状态<900 返回FAILURE
        if Gripper_status < 900:
            return py_trees.common.Status.FAILURE
        else:
            # Step 3:move up
            move_to_up_status = self.move_to_up.update()
            if move_to_up_status != py_trees.common.Status.SUCCESS:
                return py_trees.common.Status.FAILURE
            else:
                return py_trees.common.Status.SUCCESS

class MotionRobot(Node):
    def __init__(self):
        super().__init__("MotionRobot_node")
        #Service Client
        self._recognize_client = self.create_client(RecognizePickPlace,"vlm/recognize_pick_place_item")
        self._move_joint_client = self.create_client(MoveJoint,"/motion_control/move_joint")
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
        self.blackboard = py_trees.blackboard.Blackboard()
        self.get_logger().info("MotionRobot node has been initialized")
        self.tree = self.create_behavior_tree("红色方块","绿色方块")
        self.get_logger().info("Behavior tree created")

    def run_behavior_tree(self):
        self.tree.tick()
        # while rclpy.ok():
        #     pick_items = self.blackboard.get("pick_items")
        #     place_item = self.blackboard.get("place_item")
        #     if pick_items and place_item:
        #         break
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


    def vlm_pick_and_place(self, **kwargs):#识别、抓取、放置
        try:
            pick_objects = kwargs.get("object_1")
            place_object = kwargs.get("object_2")
            #MoveToCapture,Recognize,Pickup,Place
            behaviour_tree = self.create_behavior_tree(pick_objects,place_object)
            behaviour_tree.tick()
            response_text = f"下达机器人指令成功，等待机器人执行完成。"
        except Exception as error:
            self.get_logger().info(f"vlm_pick_and_place: {error}")
            return str(error)
        else:
            return response_text
    def create_behavior_tree(self,pick_objects,place_object):
        root = py_trees.composites.Sequence(name="Pick and Place Sequence", memory=False)
        blackboard = bb.Blackboard()
        blackboard.set("capture_point", [-1.2631661513427335, -0.01715903515959865, 1.3225808694947596, -0.2314122208601794, 1.5735326757108994, 0.225638744729096])  # 预设拍照位置
        blackboard.set("home_position" ,[-1.3762624598241155, 0.37081929445481526, 1.9833614483449218, 1.6306988926759154, 1.5603791271682912, 0.03204389293592429])  # 预设机械臂的初始位置
        move_to_capture = MoveJointBehavior(self,name="Move to Capture", target_joints=blackboard.get("capture_point"))
        move_to_home = MoveJointBehavior(self,name="Move to Home", target_joints=blackboard.get("home_position"))
        recognize = RecognizePickPlaceNode(self,name="Recognize Objects",pick_object_type=pick_objects,place_object_type=place_object)
        check_and_open_gripper = CheckAndOpenGripper(self,name="check_gripper",index=0,open_value=1000)
        root.add_children([move_to_capture,recognize,check_and_open_gripper,move_to_home])
         # Create the initial behavior tree and tick it
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup(timeout=15)  # Initialize the tree
        pick_items = blackboard.get("pick_items")
        place_item = blackboard.get("place_item")
        
        if pick_items and place_item:
            self.get_logger().info(f"Post-tick pick_items: {pick_items}")
            self.get_logger().info(f"Post-tick place_item: {place_item}")
        # Post-tick handler to process results after each tick
        
        # 循环遍历所有抓取项
        # pick_and_place_sequence = py_trees.composites.Sequence("PickAndPlaceSequence",False)
        # root.add_child(pick_and_place_sequence)
        # 动态创建抓取和放置的节点序列
        # Add the post-tick handler
        # if pick_items and place_item:
        #     pass
        #     # for item_index in range(len(blackboard.get("pick_items"))):
        #     #     pick_item_key = f"pick_item_{item_index}"
        #     #     # 将每个抓取项的位置存储在黑板上
        #     #     pick_item = blackboard.get("pick_items")[item_index]
        #     #     blackboard.set(pick_item_key, pick_item)

        #     #     # 移动到抓取位置节点
        #     #     move_to_pick = MoveToPickPlaceBehavior(self,name=f"MoveToPick_{item_index}", target_position=pick_item,updown=0.1)

        #     #     # 抓取节点
        #     #     pickup = PickObject(self,name=f"Pickup_{item_index}", object_position_key=pick_item_key)

        #     #     # 移动到放置位置节点
        #     #     move_to_place = MoveToPickPlaceBehavior(self,name=f"MoveToPlace_{item_index}", target_position=place_item,updown=0.1)

        #     #     # 放置节点
        #     #     place = PlaceObject(self,name=f"Place_{item_index}")

        #     #     # 为每个抓取项创建一个序列节点
        #     #     item_sequence = py_trees.composites.Sequence(name=f"PickAndPlace_{item_index}",memory=False)
        #     #     item_sequence.add_children([check_and_open_gripper,move_to_pick, pickup, move_to_place, place])
        # else:
        #     self.get_logger().info("No pick items or place item found in blackboard, skipping dynamic node creation.")
        return tree
def main():
    rclpy.init()
    node = MotionRobot()
    node.run_behavior_tree()
    rclpy.spin_once(node,timeout_sec=5.0)
    node.destroy_node()
    rclpy.shutdown()
    # behaviour_tree = arm_robot.create_behavior_tree("红色方块","绿色方块")
    # behaviour_tree.tick()


if __name__ == "__main__":
    main()
