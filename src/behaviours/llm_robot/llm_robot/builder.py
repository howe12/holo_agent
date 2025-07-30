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
import threading
import json

class ROSProxyBehaviour(py_trees.behaviour.Behaviour):
    """
    修复版：正确的ROS2节点代理行为
    """
    # 类级别的服务注册表，跟踪已创建的服务客户端
    _service_registry = {}
    def __init__(self, name, service_type,service_name,RB,WB,new_child):
        """
        初始化代理行为
        
        参数:
            name: 行为名称
            service_type: 服务接口类型
        """
        super().__init__(name=name)
        self.service_type = service_type
        self.service_name = service_name

        self.client = None
        self.response = None
        self.logger = None
        self.ros_node = None
        self.current_future = None
        self.RB = RB
        self.WB = WB
        self.new_child = new_child

        

    def setup(self, **kwargs,):
        """
        设置ROS2客户端
        """
        service_key = f"{self.service_name}"

        # 0.获取父节点实例
        self.ros_node = kwargs.get('node')  # py_trees_ros自动传递这个参数
        if not self.ros_node:
            self.ros_node = kwargs.get('ros_node')  # 后备方案  
        if not self.ros_node:
            raise RuntimeError("ROS Node instance not provided to behavior")

        self.logger = self.ros_node.get_logger()
        # self.logger.info(f"💎 开始为 '{self.name}' 设置 ROS2 服务客户端")
        # self.logger.info(f"📊 当前服务注册表: {ROSProxyBehaviour._service_registry}")
        
        # 1.设置黑板
        self.blackboard = self.attach_blackboard_client(name=f"MyBlackboard")
        self.blackboard.register_key(self.RB, access=py_trees.common.Access.READ)
        self.blackboard.register_key(self.WB, access=py_trees.common.Access.WRITE)

        # 2.创建服务客户端
        self.client = self.ros_node.create_client(
            self.service_type,
            self.service_name
        )

        # 3.等待服务可用
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.logger.error(f"❌ Service {str(self.service_name)} not available")
        # self.logger.info(f"✅ Connected to service {str(self.service_name)} ")
        ROSProxyBehaviour._service_registry[service_key] = self.client  # 注册服务
        self.logger.info(f"🎯 已经为 '{self.name}' 构建好客户端实例")
        self.logger.info(f"📊 当前服务注册表: {ROSProxyBehaviour._service_registry}")

    def initialise(self):
        """重置状态"""
        self.logger.debug(f"{self.ros_node} initialising")
        self.response = None
        self.current_future = None

    def update(self):
        """执行代理调用"""
        # 发送请求
        if self.current_future is None:
            # 准备请求
            request = self.service_type.Request()
            request.input_data = getattr(self.blackboard, self.RB, None)
            
            # self.logger.info(f"Sending request to {self.service_name}")
            self.current_future = self.client.call_async(request)
            return py_trees.common.Status.RUNNING
        
        # 等待请求完成
        if self.current_future.done():
            try:
                self.response = self.current_future.result()
                # self.logger.info(f"🎤 response内容为:{self.response}")
                
                if self.response.success:
                    ######################################################
                    # 1.清理非基础树节点
                    ######################################################
                    # a. 获取父节点（必须是Sequence或Selector）
                    parent = self.parent

                    # b. 获取当前节点在父节点子节点列表中的索引
                    child_index = parent.children.index(self)
                    current_node = parent.children[child_index]
                    current_node_name = current_node.name
                    
                    # c. 前一个节点是左侧相邻的兄弟节点
                    if child_index > 0:
                        previous_node = parent.children[child_index - 1]
                        previous_node_name = previous_node.name
                        # self.logger.info(f"上一个执行节点: {previous_node_name}")
                        # 检查是否为待清理节点
                        if previous_node_name not in ["Audio Input", "LLM Input"]:
                            try:
                                # 清理注册表
                                ROSProxyBehaviour._service_registry.pop(previous_node_name, None)
                                self.logger.info(f"💡 已注销注册表: {previous_node_name}")
                                # 通知构建器加入删除队列
                                if self.ros_node and hasattr(self.ros_node, 'subtree_node_cleanup'):
                                    self.ros_node.subtree_node_cleanup(previous_node_name)
                            except Exception as e:
                                self.logger.error(f"注销黑板键或注册表时出错: {e}")
                    # d. 前一个节点是最后的节点
                    else:
                        previous_node = parent.children[-1]
                        previous_node_name = previous_node.name
                        # self.logger.info(f"最后一个执行节点: {previous_node_name}")
                        if previous_node_name not in ["Audio Input", "LLM Input"]:
                            try:
                                # 注销黑板
                                # self.blackboard.unset(previous_node_name)
                                # self.logger.info(f"💡已注销黑板键: {previous_node_name}")
                                # 清理注册表
                                ROSProxyBehaviour._service_registry.pop(previous_node_name, None)
                                self.logger.info(f"💡 已注销注册表: {previous_node_name}")
                                # 通知构建器加入删除队列
                                if self.ros_node and hasattr(self.ros_node, 'subtree_node_cleanup'):
                                    self.ros_node.subtree_node_cleanup(previous_node_name)
                            except Exception as e:
                                self.logger.error(f"注销黑板键或注册表时出错: {e}")

                        else:
                            self.logger.info("当前是为基础节点，无前序节点")
                    
                    ######################################################
                    # 2.通过黑板设置下一个节点的输入
                    ######################################################
                    # self.logger.info(f"Service call succeeded")
                    self.logger.info(f"\033[36m 3️⃣ {current_node.name}回复内容:\n {self.response.output_data}\033[0m \n")
                    # self.blackboard.proxy_output = self.response.output_data
                    setattr(self.blackboard, self.WB, self.response.output_data)
                    self.logger.info(f"{self.blackboard}")
                    # self.logger.info(f"response.type:\n {self.response.type}")
                    # 注销黑板
                    # if current_node_name not in ["Audio Input", "LLM Input"]:
                    #     try:
                    #         self.blackboard.unregister_key(current_node_name)
                    #         self.logger.info(f"💡 已注销黑板键: {current_node_name}")
                    #     except Exception as e:
                    #         self.logger.error(f"尝试注销黑板键时出错: {e}")

                    ######################################################
                    # 3.动态添加树节点(LLM推理节点时处理)
                    ######################################################
                    if self.response.type == "task_list":
                        try:
                            self.logger.info(f"🎃 正在处理任务列表类型")
                            # 1. 解析 JSON 字符串为 Python 对象
                            configs = json.loads(self.response.json_configs)
                            # 2. 验证数据结构
                            if not isinstance(configs, list):
                                self.get_logger().error("配置格式错误: 需要列表")
                                return False
                            # 3. 遍历配置列表
                            for i, config in enumerate(configs, 1):
                                # 4. 提取值
                                server_name = config.get("server_name", f"unknown_{i}")
                                server_type = config.get("server_type", "BehavioursTree")
                                parameters = config.get("server_parameters", "")

                                # 构建队列请求字典（与srv_add_tree结构一致）
                                request_dict = {
                                    'server_name': server_name,
                                    'service_type': BehavioursTree,  # 固定服务类型
                                    'new_child': True
                                }
                                
                                # 添加到修改队列（需通过节点访问主类）
                                if self.ros_node and hasattr(self.ros_node, 'add_modification_request'):
                                    self.ros_node.add_modification_request(request_dict)
                                else:
                                    self.logger.error("无法访问修改队列！")
                        except Exception as e:
                            self.logger.error(f"处理task_list时出错: {str(e)}")
                            return py_trees.common.Status.FAILURE
                    elif self.response.type == "info":
                        self.logger.info(f"🎃 正在处理信息类型")
                        request_dict = {
                                'server_name': self.response.server_name,
                                'service_type': BehavioursTree,  # 固定服务类型
                                'new_child': True
                            }
                        # self.logger.info(f"request_dict:{request_dict}")
                        # 添加到修改队列（需通过节点访问主类）
                        if self.ros_node and hasattr(self.ros_node, 'add_modification_request'):
                            self.ros_node.add_modification_request(request_dict)
                        else:
                            self.logger.error("无法访问修改队列！") 
                    else:

                        ######################################################
                        # 新增的自我注销逻辑（从这里开始）
                        ######################################################
                        self.logger.info(f"parent_node:{self.parent.name}")

                    return py_trees.common.Status.SUCCESS
                else:
                    self.logger.warning(f"Service call failed: {self.response.output_data}")
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
        self.blackboard_name = "audio_output"
        self.first_flag  = True
        self.tree_lock = threading.Lock()  # 用于线程安全
 
        self.modification_queue = []  # 树修改请求队列
        self.modification_lock = threading.Lock()  # 队列操作锁

    def add_modification_request(self, request_dict):
        """线程安全地添加修改请求"""
        # with self.modification_lock:
        self.modification_queue.append(request_dict)
        self.process_modification_queue()
        # self.logger.info(f"添加子树请求: {request_dict['server_name']}")


    def add_tree_root(self):
        # 1.建立根节点
        root = py_trees.composites.Sequence(name="LLM BT", memory=True)

        # 2.建立子节点
        try:
            # 3.1 语音识别客户端           
            self.build_subtree_node(root,"Audio Input",BehavioursTree,"audio_input",new_child=False)
            # 3.2 llm调用客户端
            self.build_subtree_node(root,"LLM Input",BehavioursTree,"llm_input",new_child=False)
            self.logger.info(f"1️⃣ 已构建好行为树结构")
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


    def build_subtree_node(self,root_node,node_name,service_type,service_name,new_child):
        """构建树子节点实例"""    
        # 更新指定的黑板
        # RB = self.blackboard_name
        # if self.first_flag:
        #     self.first_flag = False
        #     RB = "proxy_input"
        # self.blackboard_name = service_name
        # WB = self.blackboard_name
        RB = "proxy_input"
        WB = "proxy_input"

        subtree_node = ROSProxyBehaviour(
            name=node_name,
            service_type=service_type,
            service_name=service_name,
            RB=RB,
            WB=WB,
            new_child=new_child
        )
        if new_child is False:
            root_node.add_child(subtree_node)
             
        self.logger.info(f"🎄 已添加 {str(node_name)} 子树节点")
        return subtree_node



    def setup_tree(self):
        """配置行为树"""
        try:
            # 初始化行为树
            # self.logger.info(f"开始初始化行为树")
            result_setup = self.tree.setup(node=self,timeout=5)
                
            self.logger.info("2️⃣ 行为树初始化成功！")
            return True
        except Exception as e:
            self.logger.error(f"Failed to start behavior tree: {str(e)}")
            return False


    def process_modification_queue(self):
        """在安全点（tick之间）处理树修改请求"""
        # self.logger.info(f"在进行树修改队列处理")
        if not self.modification_queue:
            return
            
        try:
            with self.modification_lock:
                # 复制当前队列并清空
                current_queue = self.modification_queue.copy()
                self.modification_queue.clear()
                
            for request in current_queue:
                # self.logger.info(f"正在处理修改: {request['server_name']}")
                
                # 创建子树节点（复用现有方法）
                new_root = py_trees.composites.Sequence(
                    name="LLM Input", 
                    memory=True
                )
                
                new_subtree_node = self.build_subtree_node(
                    new_root,
                    request['server_name'],
                    request['service_type'],
                    request['server_name'],
                    request['new_child']
                )

                
                # 动态添加到现有树
                self.tree.root.add_child(new_subtree_node)
                new_subtree_node.setup(node=self)
                # new_subtree_node.update()
                # new_subtree_node.initialise()
                
                # self.logger.info(f"已添加子树: {request['server_name']}")
                
        except Exception as e:
            self.logger.error(f"处理修改时出错: {str(e)}")

    def subtree_node_cleanup(self,node_name):
        """清理子树节点"""
        if self.tree and self.tree.root:
            # 1. 从行为树中移除节点
            for child in self.tree.root.children:
                if child.name == node_name:
                    self.tree.root.remove_child(child)
                    self.logger.info(f"💡 已移除子树节点: {node_name}")

    def tick_tree(self):
        """执行行为树tick"""
        try:
            with self.tree_lock:
                if self.tree:
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
            
        self.destroy_node()
        self.logger.info("ROS node destroyed")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # 1.创建行为树构建器
        builder = BehaviorTreeBuilder()
        root = builder.add_tree_root()
        # builder.logger.info("BehaviorTreeBuilder node created")

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