# behavior_tree/ros_proxy.py
import rclpy
from rclpy.node import Node
import py_trees
from llm_interfaces.srv import BehavioursTree

class ROSProxyBehaviour(py_trees.behaviour.Behaviour):
    """
    ROS2节点代理行为
    """
    def __init__(self, name, node_name):
        super().__init__(name=name)
        self.node_name = node_name  # 要调用的ROS2节点名称
        self.client = None
        self.response = None
        self.logger = None
        
        # 黑板书设置
        self.blackboard = self.attach_blackboard_client(name=f"{name} Blackboard")
        self.blackboard.register_key("proxy_input", access=py_trees.common.Access.READ)
        self.blackboard.register_key("proxy_output", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """
        设置ROS2客户端
        """
        
        # 从传入参数获取ROS节点实例
        self.ros_node = kwargs.get('ros_node')
        if not self.ros_node:
            raise RuntimeError("ROS Node instance not provided to behavior")
            
        self.logger = self.ros_node.get_logger()
        
        # 使用正确的节点实例创建客户端
        self.client = self.ros_node.create_client(
            srv_type=self.service_type,
            srv_name=f"/{self.node_name}/execute"
        )
        
        # 等待服务可用
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.logger.error(f"Service {self.node_name}/execute not available")
            return False
        return True

    def initialise(self):
        """重置状态"""
        self.response = None

    def update(self):
        """执行代理调用"""
        # 准备请求
        request = BehavioursTree.Request()
        request.input_data = self.blackboard.proxy_input
        
        try:
            # 异步发送请求
            future = self.client.call_async(request)
            # 等待响应（非阻塞检查）
            if future.done():
                self.response = future.result()
            else:
                return py_trees.common.Status.RUNNING
        except Exception as e:
            self.logger.error(f"Service call failed: {str(e)}")
            return py_trees.common.Status.FAILURE
        
        # 处理响应
        if self.response.success:
            self.blackboard.proxy_output = self.response.output_data
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE