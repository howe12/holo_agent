#!/usr/bin/env python3
# behavior_tree/builder.py

import py_trees
import py_trees_ros.trees
import rclpy
import sys
from rclpy.node import Node
from llm_interfaces.srv import BehavioursTree

import time
from rclpy.clock import Clock
from rclpy.time import Duration

class ROSProxyBehaviour(py_trees.behaviour.Behaviour):
    """
    修复版：正确的ROS2节点代理行为
    """
    def __init__(self, name, node_name, service_type):
        """
        初始化代理行为
        
        参数:
            name: 行为名称
            node_name: 要调用的ROS2节点名称
            service_type: 服务接口类型
        """
        super().__init__(name=name)
        self.node_name = node_name
        self.service_type = service_type

        self.subscriptions = []
        self.publishers = []
        self.services = []
        self.clients = []  # 然后在 setup 中添加客户端实例

        self.client = None
        self.response = None
        self.logger = None
        self.ros_node = None
        self.current_future = None
        
        # 黑板书设置
        self.blackboard = self.attach_blackboard_client(name=f"{name} Blackboard")
        self.blackboard.register_key("proxy_input", access=py_trees.common.Access.READ)
        self.blackboard.register_key("proxy_output", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """
        设置ROS2客户端
        """
        # 关键修复：获取父节点实例
        self.ros_node = kwargs.get('node')  # py_trees_ros自动传递这个参数
        if not self.ros_node:
            self.ros_node = kwargs.get('ros_node')  # 后备方案
        
        if not self.ros_node:
            raise RuntimeError("ROS Node instance not provided to behavior")
            
        self.logger = self.ros_node.get_logger()
        self.logger.info(f"Setting up proxy for '{self.node_name}'")
        
        
        # 创建服务客户端
        self.client = self.ros_node.create_client(
            self.service_type,
            f"audio_input"
        )
        
        # 修复 2: 将客户端添加到框架管理的列表
        self.clients.append(self.client)  # 关键！py_trees_ros 需要此引用

        # self.delay(3)
        # 等待服务可用
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.logger.error(f"Service /audio_input not available")
            return False
            
        self.logger.info(f"Connected to service /audio_input")
        return True

    def initialise(self):
        """重置状态"""
        self.logger.debug(f"{self.name} initialising")
        self.response = None
        self.current_future = None

    def update(self):
        """执行代理调用"""
        # 如果还没有发送请求
        if self.current_future is None:
            # 准备请求
            request = self.service_type.Request()
            if hasattr(self.blackboard, 'proxy_input'):
                request.input_data = self.blackboard.proxy_input
            
            self.logger.info(f"Sending request to {self.node_name}")
            self.current_future = self.client.call_async(request)
            return py_trees.common.Status.RUNNING
        
        # 检查请求是否完成
        if self.current_future.done():
            try:
                self.response = self.current_future.result()
                
                if self.response.success:
                    self.logger.info(f"Service call succeeded")
                    self.blackboard.proxy_output = self.response.output_data
                    return py_trees.common.Status.SUCCESS
                else:
                    self.logger.warning(f"Service call failed: {self.response.message}")
                    return py_trees.common.Status.FAILURE
                    
            except Exception as e:
                self.logger.error(f"Service call exception: {str(e)}")
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """清理资源"""
        self.logger.debug(f"{self.name} terminating with status {new_status}")
        self.current_future = None

    def delay(self, duration):
        # 非阻塞延迟函数
        clock = Clock()
        start_time = clock.now()
        while (clock.now() - start_time).nanoseconds / 1e9 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)


class BehaviorTreeBuilder(Node):
    """
    增强版行为树构建器
    """
    def __init__(self):
        super().__init__('behavior_tree_builder')
        self.tree = None
        self.logger = self.get_logger()
        # self.executor = rclpy.executors.SingleThreadedExecutor()
        # self.executor.add_node(self)
        self.tree_timer = None

    def build_tree(self):
        """构建行为树结构"""
        root = py_trees.composites.Sequence(name="LLM BT", memory=True)
        
        # 示例：添加音频输入节点代理
        audio_proxy = ROSProxyBehaviour(
            name="Audio Input Proxy",
            node_name="llm_audio_input",
            service_type=BehavioursTree
        )
        root.add_child(audio_proxy)
        
        # 示例：添加其他节点...
        # move_proxy = ROSProxyBehaviour(...)
        # root.add_child(move_proxy)
        
        self.logger.info("Behavior tree structure built")
        return root

    def start_tree(self):
        """启动行为树"""
        try:
            root = self.build_tree()
            if not root:
                self.logger.error("Failed to build behavior tree root")
                return False
                
            # 创建行为树实例
            self.tree = py_trees_ros.trees.BehaviourTree(
                root=root,
                node=self,
                unicode_tree_debug=True
            )

            clock = Clock()
            start_time = clock.now()
            
            timeout_duration = Duration(seconds=30) 
            
            # 循环设置行为树直到成功或超时
            setup_complete = False
            # while not self.tree.setup():
            #     # 检查超时
            #     elapsed_time = clock.now() - start_time
            #     if elapsed_time > timeout_duration:
            #         self.logger.error("Behavior tree setup timed out after 30 seconds")
            #         return False
                
            #     # 短暂休眠减少CPU使用
            #     rclpy.spin_once(self, timeout_sec=0.1)

            while not setup_complete:
                result_setup = self.tree.setup()
                self.logger.info(f"设置行为树: {result_setup}")
                # 尝试设置行为树
                if result_setup:  # 短暂阻塞
                    setup_complete = True
                    self.logger.info("Behavior tree setup completed")
                    break
                
                elapsed_time = clock.now() - start_time

                # 检查超时
                if elapsed_time > timeout_duration:
                    self.logger.error("Behavior tree setup timed out after 30 seconds")
                    return False
                
                # 短暂休眠减少CPU使用
                rclpy.spin_once(self, timeout_sec=0.1)

            # 设置行为树
            if not self.tree.setup(timeout=30):
                self.logger.error("Failed to setup behavior tree")
                return False
                
            # 创建定时器执行行为树tick
            # self.tree_timer = self.create_timer(
            #     0.1,  # 10Hz
            #     self.tick_tree
            # )
            
            self.logger.info("Behavior tree started successfully")
            return True
        except Exception as e:
            self.logger.error(f"Failed to start behavior tree: {str(e)}")
            return False

    def tick_tree(self):
        """执行行为树tick"""
        try:
            self.tree.tick()
        except Exception as e:
            self.logger.error(f"Error during tree tick: {str(e)}")

    def shutdown(self):
        """关闭行为树"""
        if self.tree:
            self.tree.shutdown()
            self.logger.info("Behavior tree shutdown complete")
            
        if self.tree_timer:
            self.destroy_timer(self.tree_timer)
            
        # self.executor.shutdown()
        self.destroy_node()
        self.logger.info("ROS node destroyed")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # 创建行为树构建器
        builder = BehaviorTreeBuilder()
        builder.logger.info("BehaviorTreeBuilder node created")
        
        # 启动行为树
        if not builder.start_tree():
            builder.logger.error("Exiting due to startup failure")
            builder.shutdown()
            return 1
            
        # 运行ROS执行器
        builder.logger.info("Entering executor spin loop")
        builder.executor.spin()
        
    except KeyboardInterrupt:
        builder.logger.warning("Program interrupted by user")
    except Exception as e:
        builder.logger.fatal(f"Critical error: {str(e)}")
    finally:
        builder.logger.info("Shutting down...")
        builder.shutdown()
        rclpy.shutdown()
        return 0


if __name__ == "__main__":
    sys.exit(main())