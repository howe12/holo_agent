#!/usr/bin/env python3
# behavior_tree/builder.py

import py_trees
import py_trees_ros.trees
import rclpy
import sys
from rclpy.node import Node
from llm_interfaces.srv import BehavioursTree,ChildTree
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
    def __init__(self, name, service_type,service_name,RB,WB):
        """
        初始化代理行为
        
        参数:
            name: 行为名称
            service_type: 服务接口类型
        """
        super().__init__(name=name)
        self.service_type = service_type
        self.service_name = service_name

        self.clients = []  # 然后在 setup 中添加客户端实例
        self.client = None
        self.response = None
        self.logger = None
        self.ros_node = None
        self.current_future = None
        self.RB = RB
        self.WB = WB
        
        # 黑板书设置
        self.blackboard = self.attach_blackboard_client(name=f"MyBlackboard")
        self.blackboard.register_key(self.RB, access=py_trees.common.Access.READ)
        self.blackboard.register_key(self.WB, access=py_trees.common.Access.WRITE)

        #  在 self 上创建动态属性
        # 读属性 (RB)
        # setattr(ROSProxyBehaviour, self.RB , 
        #         property(fget=lambda self: getattr(self.blackboard, self.RB )))
        
        # # 写属性 (WB)
        # setattr(ROSProxyBehaviour, self.WB , 
        #         property(fget=lambda self: getattr(self.blackboard, self.WB ),
        #                  fset=lambda self, value: setattr(self.blackboard, self.WB , value)))

    def setup(self, **kwargs,):
        """
        设置ROS2客户端
        """
        # 1.获取父节点实例
        self.ros_node = kwargs.get('node')  # py_trees_ros自动传递这个参数
        if not self.ros_node:
            self.ros_node = kwargs.get('ros_node')  # 后备方案  
        if not self.ros_node:
            raise RuntimeError("ROS Node instance not provided to behavior")
            
        self.logger = self.ros_node.get_logger()
        self.logger.info(f"Setting up proxy for '{self.ros_node}'")
        
        # 2.创建服务客户端
        self.client = self.ros_node.create_client(
            self.service_type,
            self.service_name
        )
        self.clients.append(self.client)  # # 将客户端添加到框架管理的列表,py_trees_ros 需要此引用

        # 3.等待服务可用
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.logger.error(f"Service {str(self.service_name)} not available")
        self.logger.info(f"Connected to service {str(self.service_name)} ")

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
            # if hasattr(self.blackboard, 'proxy_input'):
                # request.input_data = self.blackboard.proxy_input
            request.input_data = getattr(self.blackboard, self.RB, None)
            
            self.logger.info(f"Sending request to {self.ros_node}")
            self.current_future = self.client.call_async(request)
            return py_trees.common.Status.RUNNING
        
        # 检查请求是否完成
        if self.current_future.done():
            try:
                self.response = self.current_future.result()
                
                if self.response.success:
                    self.logger.info(f"Service call succeeded")
                    self.logger.info(f"response.output_data:\n {self.response.output_data}")
                    # self.blackboard.proxy_output = self.response.output_data
                    setattr(self.blackboard, self.WB, self.response.output_data)
                    self.logger.info(f"{self.blackboard}")
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

        # 1.参数初始化
        self.tree = None
        self.logger = self.get_logger()
        self.tree_timer = None
        # 创建行为树的服务端
        self.srv = self.create_service(ChildTree, "/child_tree", self.srv_add_tree)
        self.blackboard_name = "audio_output"
        self.first_flag  = True

    def srv_add_tree(self,request,response):
        """
        服务端回调函数，用于添加子行为树
        """
        root = py_trees.composites.Sequence(name="LLM BT", memory=True)

        self.logger.info(f"Received request to add tree: {request}")

        node_name = request.server_name
        server_type = BehavioursTree
        server_name = request.server_name
        add_child_tree(root,node_name,server_type,server_name)

        response.success = True
        response.output_data = "Tree added successfully"
        return response

    def add_tree_root(self):
        # 2.建立根节点
        root = py_trees.composites.Sequence(name="LLM BT", memory=True)

        # 3.建立子节点
        try:
            # 3.1 语音识别客户端           
            root = self.add_child_tree(root,"Audio Input Proxy",BehavioursTree,"audio_input")
            # 3.2 llm调用客户端
            root = self.add_child_tree(root,"LLM Input Proxy",BehavioursTree,"llm_input")
            # 3.3 语音合成客户端
            root = self.add_child_tree(root,"Audio Output Proxy",BehavioursTree,"audio_output")
            if not root:
                self.logger.error("Failed to build behavior tree root")
                return False
                
            # 创建行为树实例
            self.tree = py_trees_ros.trees.BehaviourTree(
                root=root,
                unicode_tree_debug=True
            )
        except Exception as e:
            self.logger.error(f"Failed to start behavior tree: {str(e)}")
            return False
        return root


    def add_child_tree(self,root_node,node_name,service_type,service_name):
        """建立树子节点"""
        root = root_node
        
        # 更新指定的黑板
        RB = self.blackboard_name
        if self.first_flag:
            self.first_flag = False
            RB = "proxy_input"
        self.blackboard_name = server_name
        WB = self.blackboard_name

        child_node = ROSProxyBehaviour(
            name=node_name,
            service_type=service_type,
            service_name=service_name,
            RB=RB,
            WB=WB
        )
        root.add_child(child_node)
             
        self.logger.info(f"Child node {str(node_name)} build")
        return root

    def setup_tree(self):
        """配置行为树"""
        try:
            # 初始化行为树
            self.logger.info(f"开始初始化行为树")
            result_setup = self.tree.setup(node=self,timeout=5)
                
            self.logger.info("Behavior tree setup successfully")
            return True
        except Exception as e:
            self.logger.error(f"Failed to start behavior tree: {str(e)}")
            return False

    def tick_tree(self):
        """执行行为树tick"""
        try:
            # self.tree.tick()
            self.tree.tick_tock(period_ms=1000)
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
        # 1.创建行为树构建器
        builder = BehaviorTreeBuilder()
        root = builder.add_tree_root()
        builder.logger.info("BehaviorTreeBuilder node created")

        # 2.创建黑板
        Blackboard = Client(name="MyBlackboard") 
        Blackboard.register_key("proxy_input", access=py_trees.common.Access.WRITE)
        Blackboard.set("proxy_input","actioning")
        
        # 3.配置行为树
        if not builder.setup_tree():
            builder.logger.error("Exiting due to startup failure")
            builder.shutdown()
            return 0

        # 4.执行行为树
        builder.tick_tree()
            
        # 运行ROS执行器
        builder.logger.info("Entering executor spin loop")
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