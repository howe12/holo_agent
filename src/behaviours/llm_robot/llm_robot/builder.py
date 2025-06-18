# behavior_tree/builder.py
import py_trees
import py_trees_ros.trees
import rclpy
from rclpy.node import Node
from .ros_proxy import ROSProxyBehaviour

class BehaviorTreeBuilder(Node):
    def __init__(self):
        super().__init__('behavior_tree_builder')
        self.tree = None
        
    def build_tree(self):
        """构建行为树结构"""
        root = py_trees.composites.Sequence("LLM BT", memory=True)
        
        # 添加音频输入节点代理
        audio_proxy = ROSProxyBehaviour(
            name="Audio Input Proxy",
            node_name="audio_input",
        )
        
        root.add_children([audio_proxy])
        return root
    
    def start_tree(self):
        """启动行为树"""
        self.tree = py_trees_ros.trees.BehaviourTree(
            root=self.build_tree(),
            unicode_tree_debug=True
        )
        self.tree.setup(timeout=15.0)
        
        # 设置执行定时器
        self.timer = self.create_timer(0.1, self.tick_tree)
    
    def tick_tree(self):
        """执行行为树tick"""
        self.tree.tick()
    
    def shutdown(self):
        """关闭行为树"""
        if self.tree:
            self.tree.shutdown()
        self.destroy_node()

def main():
    rclpy.init()
    node = BehaviorTreeBuilder()
    node.start_tree()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
     main()