#!/usr/bin/env python3
# 위에 내용 필수 절대 지우면 안됨!!!!

import os, signal, subprocess
import rclpy
from rclpy.node import Node
from web_control_bridge.msg import NodeManagerMsg 

# 필요하면 바꾸세요
SETUPS = [
    "/opt/ros/humble/setup.bash",
    os.path.expanduser("~/colcon_ws/install/setup.bash"),
]

def build_shell_cmd(user_cmd: str) -> str:
    src = " && ".join(f"source {s}" for s in SETUPS if os.path.exists(s))
    return f"{src} && {user_cmd}" if src else user_cmd

CMD_MAP = {
    1: ("robovision", "ros2 launch robocup_vision robocup_vision.launch.py"),
    2: ("imu", "ros2 launch ebimu e2box_imu_9dofv4.launch.py")
}

class NodeManager(Node):
    def __init__(self):
        super().__init__("node_manager_node")
        self.sub = self.create_subscription(NodeManagerMsg, "/nodemanager", self.on_cmd, 10)
        self.procs = {}

    def on_cmd(self, msg: NodeManagerMsg):
        action = msg.action.lower()

        info = CMD_MAP.get(msg.id)
        if not info:
            self.get_logger().warn(f"unknown id: {msg.id}")
            return
        
        alias, cmd = info

        if action == "start":
            self.start(alias, cmd)
        elif action == "stop":
            self.stop(alias)

    def start(self, alias: str, cmd: str):
        # 이미 실행 중이면 무시
        p = self.procs.get(alias)
        if p and (p.poll() is None):
            return

        shell_cmd = build_shell_cmd(cmd)
        p = subprocess.Popen(["bash", "-lc", shell_cmd], preexec_fn=os.setsid)
        self.procs[alias] = p
        self.get_logger().info(f"[{alias}] started (pid={p.pid}): {cmd}")

    def stop(self, alias: str):
        p = self.procs.get(alias)
        if not p:
            return
        if p.poll() is None:
            try:
                pgid = os.getpgid(p.pid)
                os.killpg(pgid, signal.SIGINT)  # Ctrl+C 효과
                self.get_logger().info(f"[{alias}] SIGINT sent (pgid={pgid})")
            except ProcessLookupError:
                pass
        # 목록에서 제거 (실제 종료는 비동기)
        self.procs.pop(alias, None)

def main():
    rclpy.init()
    node = NodeManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
