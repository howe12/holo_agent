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

该模块演示了 py_trees 库中序列（Sequence）组合行为的使用。
序列行为会按顺序依次执行其子行为，直到所有子行为成功完成，或者遇到一个失败的子行为。

.. argparse::
   :module: py_trees.demos.sequence
   :func: command_line_argument_parser
   :prog: py-trees-demo-sequence

.. graphviz:: dot/demo-sequence.dot

.. image:: images/sequence.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import sys
import time
import typing

import py_trees
import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description() -> str:
    """
    打印程序的描述和使用信息。

    如果控制台支持颜色，会使用彩色格式输出；否则以普通文本输出。

    Returns:
       程序描述字符串
    """
    content = "Demonstrates sequences in action.\n\n"
    content += (
        "A sequence is populated with 2-tick jobs that are allowed to run through to\n"
    )
    content += "completion.\n"

    if py_trees.console.has_colours:
        # 控制台支持颜色时，使用彩色横幅包装描述信息
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = banner_line
        s += console.bold_white + "Sequences".center(79) + "\n" + console.reset
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
    为 --help 选项打印一段有趣的结尾信息。

    如果控制台支持颜色，会使用彩色输出；否则返回 None。

    Returns:
       有趣的结尾信息字符串，若不支持颜色则为 None
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

    定义并配置命令行参数解析器，支持 --render 选项。

    Returns:
        配置好的参数解析器对象
    """
    parser = argparse.ArgumentParser(
        description=description(),
        epilog=epilog(),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "-r", "--render", action="store_true", help="render dot tree to file"
    )
    return parser


def create_root() -> py_trees.behaviour.Behaviour:
    """
    创建根行为及其子树。

    创建一个带有记忆功能的序列行为，并添加三个子行为，每个子行为会在两个 tick 内完成。

    Returns:
        根行为对象
    """
    root = py_trees.composites.Sequence(name="Sequence", memory=True)
    for action in ["Action 1", "Action 2", "Action 3"]:
        # 创建一个状态队列行为，前一个 tick 为 RUNNING，后一个 tick 为 SUCCESS
        rssss = py_trees.behaviours.StatusQueue(
            name=action,
            queue=[
                py_trees.common.Status.RUNNING,
                py_trees.common.Status.SUCCESS,
            ],
            eventually=py_trees.common.Status.SUCCESS,
        )
        root.add_child(rssss)
    return root


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Entry point for the demo script."""
    # 解析命令行参数
    args = command_line_argument_parser().parse_args()
    # 打印程序描述信息
    print(description())
    # 设置日志级别为 DEBUG
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    # 创建根行为
    root = create_root()

    ####################
    # Rendering
    ####################
    if args.render:
        # 如果指定了 --render 选项，渲染行为树到文件并退出程序
        py_trees.display.render_dot_tree(root)
        sys.exit()

    ####################
    # Execute
    ####################
    # 对根行为及其所有子行为进行初始化设置
    root.setup_with_descendants()
    for i in range(1, 6):
        try:
            print("\n--------- Tick {0} ---------\n".format(i))
            # 执行一次行为树的 tick 操作
            root.tick_once()
            print("\n")
            # 打印行为树的 Unicode 格式可视化信息，显示每个行为的状态
            print(py_trees.display.unicode_tree(root=root, show_status=True))
            # 暂停 1 秒
            time.sleep(1.0)
        except KeyboardInterrupt:
            # 捕获 Ctrl+C 中断信号，退出循环
            break
    print("\n")