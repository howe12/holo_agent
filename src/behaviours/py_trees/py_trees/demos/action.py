#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
A py_trees demo.

.. argparse::
   :module: py_trees.demos.action
   :func: command_line_argument_parser
   :prog: py-trees-demo-action-behaviour

.. image:: images/action.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import atexit
import multiprocessing
import multiprocessing.connection
import time
import typing

import py_trees.common
import py_trees.console as console

##############################################################################
# Classes
##############################################################################

def description() -> str:
    """
    打印程序的描述和使用信息。

    Returns:
       程序描述字符串
    """
    content = "Demonstrates the characteristics of a typical 'action' behaviour.\n"
    content += "\n"
    content += "* Mocks an external process and connects to it in the setup() method\n"
    content += (
        "* Kickstarts new goals with the external process in the initialise() method\n"
    )
    content += "* Monitors the ongoing goal status in the update() method\n"
    content += (
        "* Determines RUNNING/SUCCESS pending feedback from the external process\n"
    )

    if py_trees.console.has_colours:
        # 若控制台支持颜色，添加彩色横幅
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = banner_line
        s += console.bold_white + "Action Behaviour".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += banner_line
    else:
        s = content
    return s


def epilog() -> typing.Optional[str]:
    """
    为 --help 选项打印一段有趣的结束语。

    Returns:
       有趣的结束语消息，如果控制台不支持颜色则返回 None
    """
    if py_trees.console.has_colours:
        return (
            console.cyan
            + "And his noodly appendage reached forth to tickle the blessed...\n"
            + console.reset
        )
    else:
        return None


def command_line_argument_parser() -> argparse.ArgumentParser:
    """
    处理命令行参数。

    Returns:
        参数解析器对象
    """
    return argparse.ArgumentParser(
        description=description(),
        epilog=epilog(),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )


def planning(pipe_connection: multiprocessing.connection.Connection) -> None:
    """模拟一个（可能）长时间运行的外部进程。

    Args:
        pipe_connection: 与规划进程的连接
    """
    idle = True
    percentage_complete = 0
    try:
        while True:
            if pipe_connection.poll():
                # 若有消息，重置进度并开始新任务
                pipe_connection.recv()
                percentage_complete = 0
                idle = False
            if not idle:
                # 若任务未完成，更新进度并发送
                percentage_complete += 10
                pipe_connection.send([percentage_complete])
                if percentage_complete == 100:
                    # 任务完成，进入空闲状态
                    idle = True
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


class Action(py_trees.behaviour.Behaviour):
    """演示远程风格的动作行为。

    此行为连接到一个单独运行的进程（在 setup() 方法中启动），
    并与该子进程协作，启动任务并在每次滴答时监控任务进度，直到任务完成。
    任务运行时，行为返回 :data:`~py_trees.common.Status.RUNNING`。

    任务完成后，行为根据任务的成功或失败返回相应状态。

    关键点 - 此行为本身不应执行任何实际工作！
    """

    def __init__(self, name: str):
        """配置行为的名称。

        Args:
            name: 行为的名称
        """
        super(Action, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, **kwargs: int) -> None:
        """启动此行为将与之协作的单独进程。

        通常情况下，这个进程已经在运行。在本例中，
        setup 方法主要负责验证其存在并启动。

        Args:
            **kwargs: 可选关键字参数
        """
        self.logger.debug(
            "%s.setup()->connections to an external process" % (self.__class__.__name__)
        )
        # 创建管道用于进程间通信
        self.parent_connection, self.child_connection = multiprocessing.Pipe()
        # 创建并启动模拟外部进程的子进程
        self.planning = multiprocessing.Process(
            target=planning, args=(self.child_connection,)
        )
        # 注册在程序退出时终止子进程
        atexit.register(self.planning.terminate)
        self.planning.start()

    def initialise(self) -> None:
        """重置计数器变量并发送新任务目标。"""
        self.logger.debug(
            "%s.initialise()->sending new goal" % (self.__class__.__name__)
        )
        # 发送新任务目标
        self.parent_connection.send(["new goal"])
        self.percentage_completion = 0

    def update(self) -> py_trees.common.Status:
        """更新计数器，监控任务进度并决定新的状态。

        Returns:
            行为的新状态，可能是 RUNNING 或 SUCCESS
        """
        new_status = py_trees.common.Status.RUNNING
        if self.parent_connection.poll():
            # 若有消息，更新任务完成百分比
            self.percentage_completion = self.parent_connection.recv().pop()
            if self.percentage_completion == 100:
                # 任务完成，状态设为 SUCCESS
                new_status = py_trees.common.Status.SUCCESS
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Processing finished"
            self.logger.debug(
                "%s.update()[%s->%s][%s]"
                % (
                    self.__class__.__name__,
                    self.status,
                    new_status,
                    self.feedback_message,
                )
            )
        else:
            self.feedback_message = "{0}%".format(self.percentage_completion)
            self.logger.debug(
                "%s.update()[%s][%s]"
                % (self.__class__.__name__, self.status, self.feedback_message)
            )
        return new_status

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """此示例中无需清理操作。

        Args:
            new_status: 行为的新状态
        """
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )


##############################################################################
# Main
##############################################################################


def main() -> None:
    """演示脚本的入口点。"""
    command_line_argument_parser().parse_args()

    print(description())

    # 设置日志级别为 DEBUG
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    # 创建 Action 行为实例
    action = Action(name="Action")
    # 启动外部进程
    action.setup()
    try:
        # 模拟 12 次滴答
        for _unused_i in range(0, 12):
            action.tick_once()
            time.sleep(0.5)
        print("\n")
    except KeyboardInterrupt:
        pass
