"""CLI for the cloud task planner. Run from the SENSE repository root."""

import argparse
from contextlib import nullcontext
import json
import os
import sys

from .agent import poll_task, run_until
from .demo import DemoGateway
from .intent import TASKS, interpret, validate
from .transport import serve_gateway


def main():
    parser = argparse.ArgumentParser(description="SENSE 云端 Agent")
    parser.add_argument("instruction", nargs="?", help="自然语言指令；省略则进入交互会话")
    parser.add_argument("--gateway", action="store_true", help="接入实际 Gateway；默认使用内存模拟器")
    parser.add_argument("--task", choices=TASKS, help="直接输入结构化意图，跳过模型")
    parser.add_argument("--category")
    parser.add_argument("--region")
    parser.add_argument("--confirm-object", help="从上次返回的候选中选择 object_id")
    parser.add_argument("--status-kind", choices=("explore", "navigate"))
    parser.add_argument("--task-id")
    parser.add_argument("--example", action="store_true", help="模拟探索、查找和导航全过程")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--certfile")
    parser.add_argument("--keyfile")
    parser.add_argument("--timeout", type=float, default=20)
    parser.add_argument("--wait-seconds", type=float, default=0, help="收到 RUNNING 后继续跟踪任务的时间预算")
    args = parser.parse_args()
    if bool(args.status_kind) != bool(args.task_id):
        parser.error("--status-kind 和 --task-id 须同时提供")
    if args.example and (args.gateway or args.task or args.status_kind or args.instruction):
        parser.error("--example 独立运行，仅使用模拟数据")
    if args.status_kind and (args.task or args.instruction):
        parser.error("按任务编号查询进度时请只提供 --status-kind 和 --task-id")
    if (args.category or args.region) and not args.task:
        parser.error("--category 和 --region 用于 --task 模式")

    mode = "gateway" if args.gateway else "simulation"
    pending_choice = None

    def report(result, instruction):
        print(json.dumps({"mode": mode, "instruction": instruction, **result}, ensure_ascii=False, indent=2), flush=True)
        return 0 if result["status"] in ("SUCCESS", "RUNNING") else 1

    def execute(line, explicit=None, selection=None):
        nonlocal pending_choice
        pending_choice = None
        intent = validate(explicit) if explicit else interpret(line)
        if intent["task"] == "clarify":
            return report({"status": "NEEDS_CLARIFICATION", "message": "请说明要查询、寻找或前往哪个目标。"}, line)
        result = run_until(line, intent, gateway.call, confirm_object=selection or args.confirm_object,
                           wait_seconds=args.wait_seconds)
        if result["status"] == "NEEDS_CONFIRMATION":
            pending_choice = (line, intent)
        return report(result, line)

    try:
        if args.gateway:
            context = serve_gateway(os.environ.get("SENSE_GATEWAY_TOKEN", ""), host=args.host, port=args.port,
                                    certfile=args.certfile, keyfile=args.keyfile, timeout=args.timeout)
        else:
            context = nullcontext(DemoGateway())
        with context as gateway:
            if args.example:
                return execute("探索会议室，找到会议桌并到桌旁",
                               {"task": "find_and_navigate", "category": "会议桌", "region": "会议室"})
            if args.status_kind:
                return report(poll_task(args.status_kind, args.task_id, gateway.call), "查询已有任务")
            if args.task:
                explicit = {k: v for k, v in dict(task=args.task, category=args.category, region=args.region).items() if v}
                return execute(args.instruction or args.task, explicit)
            if args.instruction:
                return execute(args.instruction)
            print(f"运行模式 {mode}。输入自然语言指令，/state 查状态，/choose 编号选择候选，"
                  "/status explore或navigate 任务编号查进度，/stop 请求停止，/quit 退出。", flush=True)
            while True:
                try:
                    line = input("> ").strip()
                except EOFError:
                    break
                if line == "/quit":
                    break
                if line:
                    try:
                        if line.startswith("/choose "):
                            if pending_choice is None:
                                raise ValueError("当前没有待选择的候选对象")
                            execute(*pending_choice, selection=line.split(maxsplit=1)[1])
                        elif line.startswith("/status "):
                            parts = line.split()
                            if len(parts) != 3:
                                raise ValueError("用法 /status explore或navigate 任务编号")
                            report(poll_task(parts[1], parts[2], gateway.call), line)
                        elif line in ("/state", "/stop"):
                            execute(line, {"task": line[1:]})
                        elif line.startswith("/"):
                            raise ValueError("未知命令")
                        else:
                            execute(line)
                    except (OSError, ValueError) as exc:
                        print(str(exc), file=sys.stderr)
        return 0
    except (OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
