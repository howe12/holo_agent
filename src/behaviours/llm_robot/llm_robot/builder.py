#!/usr/bin/env python3
# behavior_tree/builder.py

import py_trees
import py_trees_ros.trees
import rclpy
import sys
from rclpy.node import Node
from llm_interfaces.srv import BehavioursTree
from py_trees.blackboard import Client

import time
from rclpy.clock import Clock
from rclpy.time import Duration

# # 首先创建全局黑板实例
# blackboard = py_trees.blackboard.Blackboard()
# # 在节点外部设置初始值
# blackboard.set("proxy_input", "默认输入值")  # 可以是任何数据类型


class ROSProxyBehaviour(py_trees.behaviour.Behaviour):
    """
    修复版：正确的ROS2节点代理行为
    """
    def __init__(self, name, service_type):
        """
        初始化代理行为
        
        参数:
            name: 行为名称
            service_type: 服务接口类型
        """
        super().__init__(name=name)
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
        self.blackboard = self.attach_blackboard_client(name=f"MyBlackboard")
        # self.blackboard.set("proxy_input", "action")
        self.blackboard.register_key("proxy_input", access=py_trees.common.Access.READ)
        self.blackboard.register_key("proxy_output", access=py_trees.common.Access.WRITE)

    def setup(self,service_name, **kwargs,):
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
        self.logger.info(f"Setting up proxy for '{self.ros_node}'")
        
        
        # 创建服务客户端
        self.client = self.ros_node.create_client(
            self.service_type,
            # f"audio_input"
            service_name
        )
        
        # 修复 2: 将客户端添加到框架管理的列表
        self.clients.append(self.client)  # 关键！py_trees_ros 需要此引用

        # 等待服务可用
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.logger.error(f"Service /audio_input not available")
        self.logger.info(f"Connected to service /audio_input")

    def initialise(self):
        """重置状态"""
        self.logger.debug(f"{self.ros_node} initialising")
        self.response = None
        self.current_future = None

    def update(self):
        """执行代理调用"""
        # 如果还没有发送请求
        if self.current_future is None:
            # 准备请求
            request = self.service_type.Request()
            # self.blackboard.proxy_input = "action"
            if hasattr(self.blackboard, 'proxy_input'):
                request.input_data = self.blackboard.proxy_input
            
            self.logger.info(f"Sending request to {self.ros_node}")
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
        self.logger.debug(f"{self.ros_node} terminating with status {new_status}")
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
            # 1.构建行为树架构
            root = self.build_tree()
            if not root:
                self.logger.error("Failed to build behavior tree root")
                return False
                
            # 2.创建行为树实例
            self.tree = py_trees_ros.trees.BehaviourTree(
                root=root,
                unicode_tree_debug=True
            )

            # 3.初始化行为树
            self.logger.info(f"开始初始化行为树")
            result_setup = self.tree.setup(node=self,service_name="audio_input",timeout=5)
                
            self.logger.info("Behavior tree started successfully")

            # 4.启动行为树
            self.tick_tree()
            
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
        Blackboard = Client(name="MyBlackboard") 
        Blackboard.register_key("proxy_input", access=py_trees.common.Access.WRITE)
        Blackboard.set("proxy_input","actioning")
        
        # 启动行为树
        if not builder.start_tree():
            builder.logger.error("Exiting due to startup failure")
            builder.shutdown()
            return 1
            
        # 运行ROS执行器
        builder.logger.info("Entering executor spin loop")
        # builder.executor.spin()
        rclpy.spin(builder)
        
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