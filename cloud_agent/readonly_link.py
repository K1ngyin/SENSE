"""One-request board smoke test over verified SSH. No board-side installation."""

import argparse
import inspect
import json
import os
from pathlib import Path
import re
import secrets
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor


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
            import pathlib
            root = pathlib.Path.home() / "project" / "SENSE" / "ros2_ws"
            packages = ("chassis_driver", "astra_camera_driver")
            release = pathlib.Path("/etc/os-release")
            release_lines = release.read_text().splitlines() if release.is_file() else []
            data = {"hostname": platform.node(), "architecture": platform.machine(),
                    "kernel": platform.release(), "source": "readonly_host_probe",
                    "request_id": command["request_id"],
                    "os_release": next((line.split("=", 1)[1].strip('"') for line in release_lines
                                        if line.startswith("PRETTY_NAME=")), None),
                    "ros_humble_setup_present": os.path.isfile("/opt/ros/humble/setup.bash"),
                    "workspace": str(root), "driver_sources_present": {
                        name: (root / "src" / name / "package.xml").is_file() for name in packages},
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
    parser.add_argument("--check", action="store_true", help="自动启动临时本地 Agent，并查询一次板端状态")
    parser.add_argument("--output", type=Path, help="保存 --check 成功回执的 JSON 文件")
    args = parser.parse_args()
    if not re.fullmatch(r"[a-zA-Z0-9_][a-zA-Z0-9@_.-]{0,200}", args.target):
        parser.error("请输入有效的 SSH 主机别名或 user@host")
    if not all(1 <= p <= 65535 for p in (args.port, args.remote_port)):
        parser.error("端口须在 1 至 65535 之间")
    if args.output and not args.check:
        parser.error("--output 配合 --check 使用")
    token = secrets.token_urlsafe(32) if args.check else os.environ.get("SENSE_GATEWAY_TOKEN", "")
    if not token:
        parser.error("请设置与云端相同的 SENSE_GATEWAY_TOKEN")
    program = inspect.getsource(read_one) + (
        f"\ntry:\n    read_one({token!r}, {args.remote_port}, 120)\n"
        "except Exception as exc:\n    raise SystemExit('probe failed: ' + type(exc).__name__) from None\n"
    )

    def connect(local_port):
        forward = f"127.0.0.1:{args.remote_port}:127.0.0.1:{local_port}"
        command = ["ssh", "-T", "-o", "StrictHostKeyChecking=yes", "-o", "ExitOnForwardFailure=yes",
                   "-o", "ConnectTimeout=8", "-R", forward, args.target, "python3 -IB -"]
        return subprocess.run(command, input=program.encode("utf-8")).returncode

    if not args.check:
        return connect(args.port)
    from .agent import run_task
    from .transport import serve_gateway

    with ThreadPoolExecutor(1) as pool, serve_gateway(token, port=0, timeout=300) as broker:
        pending = pool.submit(run_task, "读取板端状态", {"task": "state"}, broker.call)
        code = connect(broker.address[1])
        if code:
            broker.close()
            return code
        result = pending.result()
    output = json.dumps({"mode": "gateway", **result}, ensure_ascii=False, indent=2)
    print(output, flush=True)
    if result["status"] != "SUCCESS":
        return 1
    if args.output:
        with args.output.open("x", encoding="utf-8") as handle:
            handle.write(output + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
