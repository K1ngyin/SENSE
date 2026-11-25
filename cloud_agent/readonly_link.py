"""One-request board smoke test over verified SSH. No board-side installation."""

import argparse
import inspect
import os
import re
import subprocess
import sys


def read_one(token, port, idle_seconds):
    # Self-contained so the local CLI can send this function on SSH stdin.
    import json
    import os
    import platform
    import time
    from datetime import datetime, timezone
    from urllib import request

    client = request.build_opener(request.ProxyHandler({}))

    def post(path, payload):
        headers = {"Authorization": "Bearer " + token, "Content-Type": "application/json"}
        req = request.Request(f"http://127.0.0.1:{port}/v1/{path}", json.dumps(payload).encode(), headers)
        with client.open(req, timeout=6) as reply:
            return json.load(reply)

    end = time.monotonic() + idle_seconds
    while time.monotonic() < end:
        command = post("poll", {})["command"]
        if command is None:
            time.sleep(0.25)
            continue
        expires = datetime.fromisoformat(command["expires_at"].replace("Z", "+00:00"))
        if expires <= datetime.now(timezone.utc):
            continue
        status, data = "UNSUPPORTED", {"reason": "本验收入口只开放 get_robot_state"}
        if command["tool"] == "get_robot_state" and command["arguments"] == {}:
            status = "SUCCESS"
            data = {"hostname": platform.node(), "architecture": platform.machine(),
                    "kernel": platform.release(), "source": "readonly_host_probe",
                    "ros_humble_setup_present": os.path.isfile("/opt/ros/humble/setup.bash"),
                    "observed_at": datetime.now(timezone.utc).isoformat(),
                    "safety_ready": None, "localization_valid": None}
        post("result", {"request_id": command["request_id"], "status": status, "data": data})
        return
    raise TimeoutError("等待云端任务超时")


def main():
    parser = argparse.ArgumentParser(description="经 SSH 在板端内存中完成一次只读状态查询")
    parser.add_argument("target", help="本地已核验 SSH 主机别名或 user@host")
    parser.add_argument("--port", type=int, default=8787, help="本机云端 Agent 监听端口")
    parser.add_argument("--remote-port", type=int, default=18787, help="SSH 板端回环转发端口")
    args = parser.parse_args()
    if not re.fullmatch(r"[a-zA-Z0-9_][a-zA-Z0-9@_.-]{0,200}", args.target):
        parser.error("请输入有效的 SSH 主机别名或 user@host")
    if not all(1 <= p <= 65535 for p in (args.port, args.remote_port)):
        parser.error("端口须在 1 至 65535 之间")
    token = os.environ.get("SENSE_GATEWAY_TOKEN", "")
    if not token:
        parser.error("请设置与云端相同的 SENSE_GATEWAY_TOKEN")
    program = inspect.getsource(read_one) + f"\nread_one({token!r}, {args.remote_port}, 120)\n"
    forward = f"127.0.0.1:{args.remote_port}:127.0.0.1:{args.port}"
    command = ["ssh", "-T", "-o", "StrictHostKeyChecking=yes", "-o", "ExitOnForwardFailure=yes",
               "-o", "ConnectTimeout=8", "-R", forward, args.target, "python3 -IB -"]
    # A trace from the remote interpreter must never echo a line containing the token.
    result = subprocess.run(command, input=program.encode("utf-8"), stderr=subprocess.PIPE)
    if result.returncode:
        print("只读探针失败，请检查 SSH 主机指纹、登录、端口转发及 Agent 是否已启动。", file=sys.stderr)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
