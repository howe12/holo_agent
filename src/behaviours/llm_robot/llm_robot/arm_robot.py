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
#
# Author:  ZhangDuo @AUBO Robotics

# ROS related
from typing import List
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from std_msgs.msg import Header
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
import tf2_geometry_msgs.tf2_geometry_msgs
from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_matrix, translation_matrix, concatenate_matrices, translation_from_matrix, quaternion_from_matrix
from geometry_msgs.msg import Point,PointStamped,PoseStamped,TransformStamped
from llm_interfaces.action import ArmMoveTo
import time
from llm_interfaces.srv import ChatLLM, MoveJoint, MoveCartesian,RelativeMove
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
from std_srvs.srv import Empty
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from moveit.core.kinematic_constraints import construct_joint_constraint
from scipy.spatial.transform import Rotation as R
import copy
import numpy as np
from llm_robot.utils import adjust_tool_orientation_xyz
# LLM related
import json
import os
from llm_config.user_config import UserConfig

import tf_transformations as tf

from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from rclpy.executors import MultiThreadedExecutor

# Global Initialization
config = UserConfig()

class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')
        self.joint_order = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
        self.joint_positions = [0] * len(self.joint_order)  # 初始化数组
        # self.data_received = False
        self.logger = self.get_logger()

    def joint_state_callback(self, msg):
        self.logger.info("---------------------获取关节状态---------------")
        # 创建字典以便通过关节名称查找位置
        joint_position_dict = dict(zip(msg.name, msg.position))
        # 按照预定义顺序提取关节位置
        for i, joint_name in enumerate(self.joint_order):
            if joint_name in joint_position_dict:
                self.joint_positions[i] = joint_position_dict[joint_name]
            else:
                self.get_logger().warn(f'Joint name {joint_name} not found in JointState message')
        
        self.get_logger().info(f'Current joint positions: {self.joint_positions}')
        self.data_received = True  # 数据接收完毕，设置标志
        

    def get_current_joint_states(self):
        # 临时订阅当前机械臂关节信息
        self.subscription = self.create_subscription(JointState,'/joint_states',self.joint_state_callback,1)
        self.subscription  # 防止回收未使用变量
        self.data_received = False

        # 等待回调函数获取到数据（简单的等待机制）
        timeout = 5  # 设置超时时间为5秒
        while not self.data_received and timeout > 0 :# 数据接收完毕，设置标志 and timeout > 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            timeout -= 1
        self.destroy_subscription(self.subscription)# 销毁订阅器以停止接收消息
        # self.subscription.destroy()  
        return self.joint_positions

class ArmRobot(Node):
    def __init__(self):
        super().__init__("arm_robot")
        self.logger = self.get_logger()


        self.aubo = MoveItPy() # 创建 MoveItPy 实例
        self.joint_state_listener = JointStateListener() # 创建 JointStateListener 实例
        self.aubo_arm = self.aubo.get_planning_component("aubo_arm") # 获取 Aubo 机械臂的运动规划组件
        self.robot_model = self.aubo.get_robot_model()
        self.robot_state = RobotState(self.robot_model)

        self.tf_buffer = Buffer()   # 创建TF坐标监听
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = tf2_ros.TransformBroadcaster(self) # 创建TF坐标变换广播

        self.logger.info("MoveItPy instance created")

        #Service Server
        self._move_joint_server = self.create_service(MoveJoint,'/motion_control/move_joint',self.move_joint_callback)
        self._move_cartesian_server = self.create_service(MoveCartesian,'motion_control/move_cartesian',self.move_cartesian_callback)
        self._move_cartesian_euler_server = self.create_service(MoveCartesian,'motion_control/move_cartesian_euler',self.move_cartesian_euler_callback)
        self._move_cartesian_quat_server = self.create_service(MoveCartesian,'motion_control/move_cartesian_quat',self.move_cartesian_quat_callback)
        self._move_cartesian_object_frame_point_server = self.create_service(MoveCartesian,'motion_control/move_cartesian_objetc_frame_point',self.move_cartesian_objetc_frame_point_callback)
        self._move_relative_server = self.create_service(RelativeMove,'motion_control/realtive_move',self.move_relative_callback)

        # Server for function call
        self.function_call_server = self.create_service(ChatLLM, "/ChatLLM_function_call_service", self.function_call_callback)
        # Node initialization log
        self.get_logger().info("ArmRobot node has been initialized")

        base_link="aubo_base"   
        object_link="tool0"        
        self.transform_stamped = None
        # self.joint_timer = self.create_timer(1, lambda:self.get_trans(base_link,object_link))

    # 坐标转换
    def get_trans(self,base_link="aubo_base",object_link="tool0"):
        while rclpy.ok():
            try:
                # 尝试查找变换
                transform_stamped = self.tf_buffer.lookup_transform(base_link, object_link, Time())
                # 如果成功查找到变换，输出并退出循环
                self.get_logger().info(f'找到变换关系: \n{transform_stamped}')
                return transform_stamped
            except Exception as e:
                # 如果查找失败，捕获异常，等待并重试
                self.get_logger().warn('Transform not available yet, retrying...')
            
            # 控制循环频率，避免过于频繁的查询
            # rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(1)
        '''
        time.sleep(10)
        
        try:
            # 获取变换
            self.transform_stamped = self.tf_buffer.lookup_transform(base_link, object_link, Time() )
            # self.get_logger().info(f"****** Transform Stamped: {self.transform_stamped} ******")
            return True, self.transform_stamped
        except Exception as e:
            self.get_logger().info(f"{base_link} to {object_link} transform is not available!")
            return False, None
        '''


    # 机械臂运动规划与执行
    def plan_and_execute(self,robot,planning_component,logger,single_plan_parameters=None,multi_plan_parameters=None,sleep_time=0.0,):
        '''
        机械臂规划与执行
        @return: True 为成功到达, False 为失败
        '''
        logger.info(f"\033[36m 【状态】： 机械臂规划运动中\033[0m")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(single_plan_parameters=single_plan_parameters)
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            logger.info(f"\033[36m 【状态】： 成功规划\033[0m")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
            return True
        else:
            logger.info(f"\033[31m 【状态】： 规划失败\033[0m")
            return False

    def move_by_joint_positions(self,joint_positions: List[float])-> bool:
        if len(joint_positions) != 6:
            raise ValueError("The length of joint_positions must be 6.")
        self.aubo_arm.set_start_state_to_current_state()
        self.robot_state.set_joint_group_positions("aubo_arm",joint_positions)
        joint_constraint = construct_joint_constraint(
                robot_state=self.robot_state,
                joint_model_group=self.aubo.get_robot_model().get_joint_model_group("aubo_arm"),
            )
        self.aubo_arm.set_goal_state(motion_plan_constraints=[joint_constraint])
        result = self.plan_and_execute(self.aubo, self.aubo_arm, sleep_time=3.0)
        return result

    def move_by_pose_euler(self,x,y,z,rx,ry,rz,tcp="wrist3_Link")-> bool:
        """x,y,z -> m  rx,ry,rz -> rad"""
        if None in (x, y, z, rx, ry, rz):
            self.get_logger().error("Error: Missing one or more required arguments.")
            return False
        try:
            pose_goal = PoseStamped()
            # rot = R.from_euler('xyz', [rx, ry, rz], degrees=False)
            # rot_quat = rot.as_quat()
            rot_quat = quaternion_from_euler(rx,ry,rz)
            pose_goal.header.frame_id = "aubo_base"
            pose_goal.pose.orientation.x = rot_quat[0]
            pose_goal.pose.orientation.y = rot_quat[1]
            pose_goal.pose.orientation.z = rot_quat[2]
            pose_goal.pose.orientation.w = rot_quat[3]
            pose_goal.pose.position.x = x
            pose_goal.pose.position.y = y
            pose_goal.pose.position.z = z
            self.aubo_arm.set_goal_state(pose_stamped_msg = pose_goal, pose_link = tcp)
            result = self.plan_and_execute(self.aubo, self.aubo_arm, sleep_time=3.0)
            return result
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            return False
    def move_by_pose_stamped(self,pose_stamped: PoseStamped,tcp="wrist3_Link")-> bool:
        self.aubo_arm.set_goal_state(pose_stamped_msg = pose_stamped, pose_link = tcp)
        result = self.plan_and_execute(self.aubo, self.aubo_arm, sleep_time=3.0)
        return result

    def move_by_pose_orientation(self,x,y,z,ori_x,ori_y,ori_z,ori_w,tcp="wrist3_Link")-> bool:
        """x,y,z -> m  """
        if None in (x, y, z, ori_x, ori_y, ori_z, ori_w):
            self.get_logger().error("Error: Missing one or more required arguments.")
            return False
        try:
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "aubo_base"
            pose_goal.pose.orientation.x = ori_x
            pose_goal.pose.orientation.y = ori_y
            pose_goal.pose.orientation.z = ori_z
            pose_goal.pose.orientation.w = ori_w
            pose_goal.pose.position.x = x
            pose_goal.pose.position.y = y
            pose_goal.pose.position.z = z
            self.aubo_arm.set_goal_state(pose_stamped_msg = pose_goal, pose_link = tcp)
            result = self.plan_and_execute(self.aubo, self.aubo_arm, sleep_time=3.0)
            return result
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            return False

    def move_by_object_frame_point(self,object_point: Point,base2cam_transform: TransformStamped,object_frame="camera_color_optical_frame",use_entry_point=False,entry_point_height=0.1,tcp="tool0") -> bool:
        if None in (object_point, use_entry_point, entry_point_height) or '' in(object_frame,tcp):
            self.get_logger().error("Error: Missing one or more required arguments.")
            return False
        self.aubo_arm.set_start_state_to_current_state()
        self.base2cam_transform = base2cam_transform
        angles = [3.14, 0., 0.]
        # rot = R.from_euler('xyz', angles, degrees=False)
        # rot_quat = rot.as_quat()
        rot_quat = quaternion_from_euler(*angles,axes='sxyz')
        current_quaternion = self.aubo_arm.get_start_state().get_pose(tcp).orientation
        object_posestamped = PoseStamped()
        object_posestamped.header.frame_id = object_frame
        object_posestamped.pose.position = object_point
        # object_posestamped.pose.orientation = current_quaternion
        object_posestamped.pose.orientation.x = rot_quat[0]
        object_posestamped.pose.orientation.y = rot_quat[1]
        object_posestamped.pose.orientation.z = rot_quat[2]
        object_posestamped.pose.orientation.w = rot_quat[3]
        try:
            # if start_state_joint_positions is not None:#TODO
            #     # Step 1: 设置拍照姿态的关节位置
            #     self.robot_state.set_joint_group_positions("aubo_arm", start_state_joint_positions)
            #     # 计算TCP在aubo_base坐标系下的位置
            #     end_effector_pose = self.aubo_arm.get_start_state().get_pose(tcp)

            #     # Step 2: 获取camera_color_optical_frame相对于TCP的变换
            #     # 这里需要提前知道相机相对于TCP的固定变换
            #     camera_to_tcp_transform = self.tf_buffer.lookup_transform(
            #         tcp, object_frame, Time(),timeout=Duration(seconds=1))

            #     # Step 3: 生成 camera_color_optical_frame 相对于 aubo_base 的 TransformStamped
            #     self.initial_transform = TransformStamped()
            #     self.initial_transform.header.frame_id = 'aubo_base'
            #     self.initial_transform.child_frame_id = object_frame

            #     # 将 end_effector_pose 与 camera_to_tcp_transform 合并来计算 initial_transform
            #     self.initial_transform.transform.translation.x = (
            #         end_effector_pose.position.x + camera_to_tcp_transform.transform.translation.x)
            #     self.initial_transform.transform.translation.y = (
            #         end_effector_pose.position.y + camera_to_tcp_transform.transform.translation.y)
            #     self.initial_transform.transform.translation.z = (
            #         end_effector_pose.position.z + camera_to_tcp_transform.transform.translation.z)
            #     # 旋转部分用四元数相乘计算
            #     end_effector_quat = [
            #         end_effector_pose.orientation.x,
            #         end_effector_pose.orientation.y,
            #         end_effector_pose.orientation.z,
            #         end_effector_pose.orientation.w
            #     ]
            #     camera_quat = [
            #         camera_to_tcp_transform.transform.rotation.x,
            #         camera_to_tcp_transform.transform.rotation.y,
            #         camera_to_tcp_transform.transform.rotation.z,
            #         camera_to_tcp_transform.transform.rotation.w
            #     ]

            #     combined_quat = quaternion_multiply(end_effector_quat, camera_quat)
            #     self.initial_transform.transform.rotation.x = combined_quat[0]
            #     self.initial_transform.transform.rotation.y = combined_quat[1]
            #     self.initial_transform.transform.rotation.z = combined_quat[2]
            #     self.initial_transform.transform.rotation.w = combined_quat[3]
            # else:
            trans = self.base2cam_transform if self.base2cam_transform.header.frame_id != '' else self.tf_buffer.lookup_transform('aubo_base',object_frame,Time(),timeout=Duration(seconds=1))
            """geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1725024851,
              nanosec=731609297), frame_id='aubo_base'), child_frame_id='camera_color_optical_frame', 
              transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.04127928376895684, y=-0.44033305530385813, z=0.3875937991806053), 
              rotation=geometry_msgs.msg.Quaternion(x=-0.022743060431830605, y=0.9997328107980704, z=-0.0007235040066485933, w=0.004066541274357892)))"""
            transformed_posestamped = tf2_geometry_msgs.do_transform_pose_stamped(object_posestamped, trans)
            adjusted_quat_msg = adjust_tool_orientation_xyz(current_quaternion,transformed_posestamped.pose.orientation)
            transformed_posestamped.pose.orientation = adjusted_quat_msg
            if use_entry_point:
                transformed_posestamped.pose.position.z += entry_point_height
                self.aubo_arm.set_goal_state(pose_stamped_msg=transformed_posestamped, pose_link=tcp)
                if self.plan_and_execute(self.aubo, self.aubo_arm):
                    transformed_posestamped.pose.position.z -= entry_point_height
                    self.aubo_arm.set_goal_state(pose_stamped_msg=transformed_posestamped, pose_link=tcp)
                else:
                    return False
            else:
                self.aubo_arm.set_goal_state(pose_stamped_msg=transformed_posestamped, pose_link=tcp)
            result = self.plan_and_execute(self.aubo, self.aubo_arm)
            return result
        except Exception as e:
            self.get_logger().warn('Could not transform pose: {}'.format(str(e)))
            return False
    def move_by_relative(self,dx,dy,dz,drx,dry,drz,reference_frame=0,tcp='wrist3_Link')-> bool:
        '''reference_frame: 0->base 1->eef'''
        if None in (dx,dy,dz,drx,dry,drz) or '' in(reference_frame,tcp):
            self.get_logger().error("Error: Missing one or more required arguments.")
            return False
        current_pose = self.aubo_arm.get_start_state().get_pose(tcp)
        current_position = [
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z
        ]
        current_orientation = [
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w
            ]
        if reference_frame == 0:#基于基坐标相当运动
            new_position = [
                current_position[0] + dx,
                current_position[1] + dy,
                current_position[2] + dz
            ]
            new_orientation = quaternion_multiply(quaternion_from_euler(drx, dry, drz),current_orientation)
        else:#基于末端TCP相对运动
            # 在末端坐标系下，首先计算相对运动的变换矩阵
            relative_translation = translation_matrix([dx, dy, dz])
            relative_rotation = quaternion_matrix(quaternion_from_euler(drx, dry, drz))
            # 当前姿态的变换矩阵
            current_transform = quaternion_matrix(current_orientation)
            current_transform[:3, 3] = current_position
            # 将相对运动变换到基坐标系
            new_transform = concatenate_matrices(current_transform, relative_translation, relative_rotation)
            # 提取新位置和方向
            new_position = translation_from_matrix(new_transform)
            new_orientation = quaternion_from_matrix(new_transform)
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.frame_id = "aubo_base"
        pose_stamped.pose.position.x = new_position[0]
        pose_stamped.pose.position.y = new_position[1]
        pose_stamped.pose.position.z = new_position[2]
        pose_stamped.pose.orientation.x = new_orientation[0]
        pose_stamped.pose.orientation.y = new_orientation[1]
        pose_stamped.pose.orientation.z = new_orientation[2]
        pose_stamped.pose.orientation.w = new_orientation[3]
        self.aubo_arm.set_goal_state(pose_stamped_msg=pose_stamped, pose_link=tcp)
        multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
                self.aubo, ["ompl_rrtc", "pilz_lin"]
            )
        # plan to goal
        result = self.plan_and_execute(self.aubo, self.aubo_arm, multi_plan_parameters=multi_pipeline_plan_request_params,
                        sleep_time=3.0)
        return result

    def move_joint_callback(self, request, response):
        target_position = request.angles
        success = self.move_by_joint_positions(target_position)
        response.success = success
        return response
    def move_cartesian_callback(self, request, response):
        pose_stamped = request.pose_stamped
        pose_link = request.pose_link
        success = self.move_by_pose_stamped(pose_stamped,pose_link)
        response.success = success
        return response
    def move_cartesian_euler_callback(self, request, response):
        pose_euler = request.pose_euler
        pose_link = request.pose_link
        success = self.move_by_pose_euler(*pose_euler, tcp=pose_link)
        response.success = success
        return response
    def move_cartesian_quat_callback(self, request, response):
        pose_quat = request.pose_quat
        pose_link = request.pose_link
        success = self.move_by_pose_orientation(*pose_quat,tcp=pose_link)
        response.success = success
        return response
    def move_cartesian_objetc_frame_point_callback(self, request: MoveCartesian.Request, response):
        base2cam_transform = request.base2cam_transform
        object_point = request.object_point
        pose_link = request.pose_link
        object_frame = request.object_frame
        use_entry_point = request.use_entry_point
        entry_point_height = request.entry_point_height
        success = self.move_by_object_frame_point(object_point,base2cam_transform,object_frame,use_entry_point,entry_point_height,tcp=pose_link)
        response.success = success
        return response
    def move_relative_callback(self, request, response):
        success = self.move_by_relative(request.dx,request.dy,request.dz,request.drx,request.dry,request.drz,request.reference_frame,tcp=request.pose_link)
        response.success = success
        return response


    def forward_joint_movement(self,joint_number,angle,clockwise="True"):
        '''
        控制单关节移动方法,joint_number为控制的关节，angle为角度值，clockwise为是否顺时针运动
        '''

        # 角度转弧度
        pi = 3.1415826
        radian = (angle*pi)/180
        
        # 判断是否为顺时针旋转
        if clockwise == "True":
            self.logger.info("---------------------顺时针运动---------------")
            radian = -1*radian
        else:
            self.logger.info("---------------------逆时针运动---------------")
        #  初始化关节目标位置数组
        target_positions = [0]*6 

        # 获取当前关节状态
        current_joint_positions = self.joint_state_listener.get_current_joint_states()
        self.logger.info(f'Current joint positions: {current_joint_positions}')
        # self.joint_state_listener.destroy_node()

        # 赋取当前各个关节角度值
        for i, position in enumerate(current_joint_positions):
            target_positions[i] = position

        # 更新指定关节角度值
        target_positions[joint_number-1] +=  radian
        self.logger.info(f'Target joint positions: {target_positions}')
        

        # set plan start state to current state
        self.aubo_arm.set_start_state_to_current_state()

        # set constraints message
        joint_values = {
            "shoulder_joint": target_positions[0],
            "upperArm_joint": target_positions[1],
            "foreArm_joint": target_positions[2],
            "wrist1_joint": target_positions[3],
            "wrist2_joint": target_positions[4],
            "wrist3_joint": target_positions[5],
        }
        
        self.robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
                self.robot_state,
                joint_model_group=self.aubo.get_robot_model().get_joint_model_group("aubo_arm"),
        )
        self.aubo_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        # plan to goal
        self.plan_and_execute(self.aubo, self.aubo_arm, self.logger)
    # 特定位姿移动
    def specific_posture_movement(self,posture_name):

        # 零位
        self.zero = {
            "foreArm_joint": 0.0,"wrist1_joint": 0.0,"upperArm_joint": 0.0,"wrist2_joint": 0.0,"wrist3_joint": 0.0,"shoulder_joint": 0.0,
        }
        # home
        self.home = {
            "foreArm_joint": -1.541861979076825,"wrist1_joint": -3.6157999327406294e-05,"upperArm_joint": -9.690248793922365e-05,"wrist2_joint": -1.600751241570525,"wrist3_joint": -2.6292469538748267e-05,"shoulder_joint": -9.398494474589825e-05,
        }
        # 在车上的初始姿态
        self.ready_in_car = {
            "foreArm_joint": -2.4108334358394976,"wrist1_joint": 0.19090251921361948,"upperArm_joint": -1.034534295735007,"wrist2_joint": -1.5654776861404676,"wrist3_joint": -0.10093312750890894,"shoulder_joint": 1.5002197993246449,
        }
        # 抓取姿态
        self.ready_values = {
            "foreArm_joint": -0.9656377251059474,"wrist1_joint": 0.26844829995519176,"upperArm_joint": 0.33847095519177534,"wrist2_joint": -1.5632255169254201,"wrist3_joint": -0.30126247877899875,"shoulder_joint": 1.2959143056618647,
        }
        self.ready2_values = {
            "foreArm_joint": -0.2885711017691204,"wrist1_joint": 0.7373690042739373,"upperArm_joint": 0.5566415948364458,"wrist2_joint": -1.5620737561216333,"wrist3_joint": -0.3733832460530537,"shoulder_joint": 1.2129104992004374,
        }
        self.ready_left = {
            "foreArm_joint": -1.2078889688170489,"wrist1_joint": 0.3579811963234353,"upperArm_joint": -0.013439654729536009,"wrist2_joint": -1.5887366519264905,"wrist3_joint": 0.053964028488246114,"shoulder_joint": 1.5866238677768694,
        }
        self.ready_right = {
            "foreArm_joint": -1.0936519036159928,"wrist1_joint": 0.3630834233236494,"upperArm_joint": 0.0958198964245822,"wrist2_joint": -1.5684707970190979,"wrist3_joint": -0.7777613290232959,"shoulder_joint": 0.7547371170316755,
        }
        # 抓取后（前）姿态
        self.catch_values = {
            "foreArm_joint": -1.9574101499793466,"wrist1_joint": 0.2739686821516851,"upperArm_joint": -0.6628273384951079,"wrist2_joint": -1.55778949937379,"wrist3_joint": -0.05559996899298767,"shoulder_joint": 1.5361444659242836,
        }
        # 放置物体姿态
        self.place_values = {
            "foreArm_joint": -0.965094856956392,"wrist1_joint": -0.35666070622992196,"upperArm_joint": 0.9411279617591244,"wrist2_joint": -1.624441236843877,"wrist3_joint": -0.18545916560591436,"shoulder_joint": 1.3522111999819844,
        }
        self.place2_values = {
            "foreArm_joint": -0.9197213501320577,"wrist1_joint": -0.045931047086039824,"upperArm_joint": 0.7063044748240355,"wrist2_joint": -1.5742185969539186,"wrist3_joint": -0.20890886884988916,"shoulder_joint": 1.3430447979161793,
        }

        self.posture = {
            "zero": self.zero,
            "home": self.home,
            "ready_in_car": self.ready_in_car,
            "ready": self.ready_values,
            "ready2": self.ready2_values,
            "ready_left": self.ready_left,
            "ready_right": self.ready_right,
            "catch": self.catch_values,
            "place": self.place_values,
            "place2": self.place2_values,
        }

        # 根据传入的期望位姿设定各个关节的角度值
        target_positions = self.posture[posture_name]

        # 将目标关节位置数组设置为当前机器人状态的关节位置
        # 这一步设置了机器人希望达到的目标关节位置
        self.robot_state.joint_positions = target_positions

        # 构造一个关节约束
        # 该约束基于当前的机器人状态和关节模型组（"aubo_arm" 是机械臂的关节组）
        # 这里使用 get_robot_model().get_joint_model_group("aubo_arm") 获取关节组模型
        joint_constraint = construct_joint_constraint(
            self.robot_state,
            joint_model_group=self.aubo.get_robot_model().get_joint_model_group("aubo_arm"),
        )

        # 将构造的关节约束设置为运动规划的目标状态
        # motion_plan_constraints 参数是一个列表，包含一个或多个运动规划的约束条件
        self.aubo_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        self.plan_and_execute(self.aubo, self.aubo_arm, self.logger)


    def transform_stamped_to_pose_stamped(self,transform_stamped):
        '''
        转换 transform_stamped 为 pose_stamped 格式
        '''
        # 创建一个空的 PoseStamped 消息
        pose_stamped = PoseStamped()

        # 复制 header
        pose_stamped.header = transform_stamped.header

        # 将 translation 和 rotation 从 TransformStamped 复制到 PoseStamped 的 pose 中
        pose_stamped.pose.position.x = transform_stamped.transform.translation.x
        pose_stamped.pose.position.y = transform_stamped.transform.translation.y
        pose_stamped.pose.position.z = transform_stamped.transform.translation.z

        pose_stamped.pose.orientation.x = transform_stamped.transform.rotation.x
        pose_stamped.pose.orientation.y = transform_stamped.transform.rotation.y
        pose_stamped.pose.orientation.z = transform_stamped.transform.rotation.z
        pose_stamped.pose.orientation.w = transform_stamped.transform.rotation.w

        return pose_stamped
    
    # 转换 transform_stamped 为 matrix 格式
    def transform_stamped_to_matrix(self,transform_stamped):
        translation = transform_stamped.transform.translation
        rotation = transform_stamped.transform.rotation
        
        # 将四元数转换为旋转矩阵
        rotation_matrix = tf.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        
        # 将平移部分添加到齐次矩阵
        translation_matrix = tf.translation_matrix([translation.x, translation.y, translation.z])
        
        # 将旋转和平移部分组合成一个 4x4 齐次矩阵
        transform_matrix = tf.concatenate_matrices(translation_matrix, rotation_matrix)
        
        return transform_matrix


    def matrix_to_transform_stamped(self,matrix, frame_id, child_frame_id):
        '''
        转换 matrix 为 transform_stamped 格式
        '''
        transform_stamped = TransformStamped()
        
        # 提取平移和旋转
        translation = tf.translation_from_matrix(matrix)
        rotation = tf.quaternion_from_matrix(matrix)
        
        # 填充 TransformStamped
        transform_stamped.header.frame_id = frame_id
        transform_stamped.child_frame_id = child_frame_id
        transform_stamped.transform.translation.x = translation[0]
        transform_stamped.transform.translation.y = translation[1]
        transform_stamped.transform.translation.z = translation[2]
        transform_stamped.transform.rotation.x = rotation[0]
        transform_stamped.transform.rotation.y = rotation[1]
        transform_stamped.transform.rotation.z = rotation[2]
        transform_stamped.transform.rotation.w = rotation[3]
        
        return transform_stamped

    def get_translation_from_transform(self,transform_stamped):
        translation = transform_stamped.transform.translation
        return np.array([translation.x, translation.y, translation.z])

    def adjust_position_and_orientation(self,trans_x=None, trans_y=None,trans_z=None, rotation_x=None,rotation_y=None,rotation_z=None):
        '''
        逆运动学控制机械臂
        '''
        
        # 设置机械臂的起始位置为当前状态
        self.aubo_arm.set_start_state_to_current_state()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "aubo_base"

        # 初始 3x3 旋转矩阵（通常为单位矩阵）,和 4x4 同质变换矩阵
        rotation_mat = np.eye(3)
        transform_matrix = np.eye(4)

        # 处理 rotation_x, rotation_y, rotation_z 生成旋转矩阵，并进行矩阵累积
        if rotation_x is not None:
            rot_mat_delta_x = R.from_euler('x', np.deg2rad(rotation_x)).as_matrix()[:3, :3]
            rotation_mat = np.dot(rotation_mat, rot_mat_delta_x)
        if rotation_y is not None:
            rot_mat_delta_y = R.from_euler('y', np.deg2rad(rotation_y)).as_matrix()[:3, :3]
            rotation_mat = np.dot(rotation_mat, rot_mat_delta_y)
        if rotation_z is not None:
            rot_mat_delta_z = R.from_euler('z', np.deg2rad(rotation_z)).as_matrix()[:3, :3]
            rotation_mat = np.dot(rotation_mat, rot_mat_delta_z)
        
        # 将旋转矩阵放入 4x4 变换矩阵的左上角 
        transform_matrix[:3, :3] = rotation_mat

        # 处理平移变化，并添加到矩阵中，生成最终的转换矩阵
        if trans_x is None:
            trans_x = 0.0
        if trans_y is None:
            trans_y = 0.0
        if trans_z is None:
            trans_z = 0.0          
        transform_matrix[:3, 3] = np.array([trans_x, trans_y, trans_z])
        self.get_logger().info(f"\033[32m 转换矩阵 : \n{transform_matrix}\033[0m")
        transform_stamped = self.matrix_to_transform_stamped(transform_matrix,"tool0","target") # 矩阵转换成transform_stamped格式
        
        # 发布期望位置tf坐标系信息
        cube_tf = TransformStamped()
        cube_tf.header.stamp = self.get_clock().now().to_msg()
        cube_tf.header.frame_id = "tool0"
        cube_tf.child_frame_id = "target"
        cube_tf.transform.translation = transform_stamped.transform.translation
        cube_tf.transform.rotation = transform_stamped.transform.rotation
        self.tf_pub.sendTransform(cube_tf)
        time.sleep(3)


        # 获取期望位置与aubo_base的转换关系
        tf_trans = self.get_trans(object_link="target")
        # translation = tf_trans.transform.translation
        # rotation = tf_trans.transform.rotation
        # self.get_logger().info(f"平移Translation: \n{translation.x}, {translation.y}, {translation.z}")
        # self.get_logger().info(f"旋转Rotation: \n{rotation.x}, {rotation.y}, {rotation.z}, {rotation.w}")

        # 设置目标，机械臂规划运动
        pose_stamped_msg = self.transform_stamped_to_pose_stamped(tf_trans) # 矩阵转换成transform_stamped格式
        self.aubo_arm.set_goal_state(pose_stamped_msg=pose_stamped_msg, pose_link="tool0")
        self.plan_and_execute(self.aubo, self.aubo_arm, self.logger)
        

    # 休眠操作
    def delay(self, duration):
        # 非阻塞延迟函数
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('----------over-----------')

    def function_call_callback(self, request, response):
        """
        处理函数调用请求的回调函数。
        根据请求中的函数调用信息逐一调用相应的函数，并将每个函数的执行结果添加到响应中。
        """
        # self.get_logger().info(f"接收到方法请求: {request}")
        # 将请求中的 JSON 字符串解析为列表形式的函数调用请求
        reqs = json.loads(request.request_text)
        # 存储所有函数调用的返回结果
        rets = []

        # 遍历所有的函数调用请求
        for req in reqs:
            # 获取函数名称和参数
            function_name = req["name"]  # 函数名称
            function_args = req["params"]  # 函数参数
            
            # 如果函数参数是字符串，将其解析为字典
            if isinstance(function_args, str):
                try:
                    function_args = json.loads(function_args)
                    self.get_logger().info(f"函数参数: {function_args}")
                except json.JSONDecodeError as e:
                    self.get_logger().info(f"参数解析失败: {e}")
                    rets.append(f"参数解析失败: {e}")
                    continue

            # 根据函数名称获取类中的函数对象
            func_obj = getattr(self, function_name)

            try:
                # 尝试调用函数，并传递参数
                function_execution_result = func_obj(**function_args)
            except Exception as error:
                # 如果调用过程中发生异常，则记录错误日志，并将错误信息作为返回值
                self.get_logger().info(f"调用函数失败: {error}")
                ret = str(error)
            else:
                # 如果函数调用成功，将函数执行结果转换为字符串作为返回值    
                ret = str(function_execution_result)

            # 将当前函数调用的结果添加到返回结果列表中
            rets.append(ret)

        # 将所有函数调用的结果列表转换为字符串，并赋值给响应的 response_text 字段
        response.response_text = str(rets)
        # 返回响应对象
        return response


    # def arm_zero(self, **kwargs):
    #     try:
    #         result = self.move_by_joint_positions([0.0] * 6)
    #         response_text = "所有关节回到了原点" if result else "机械臂位置归零执行失败！"
    #     except Exception as error:
    #         self.get_logger().info(f"arm_zero: {error}")
    #         return str(error)
    #     else:
    #         return response_text

def main():
    rclpy.init()
    arm_robot = ArmRobot()

    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(arm_robot)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        arm_robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
