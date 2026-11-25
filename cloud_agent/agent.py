"""High-level task orchestration. All positions and motion decisions belong to the gateway."""

import math
import re
import time
from datetime import datetime


TASK_FIELDS = {
    "state": set(),
    "exploration_status": set(),
    "map": set(),
    "query": {"category", "region"},
    "explore": {"region"},
    "find": {"category", "region"},
    "navigate": {"category", "region"},
    "find_and_navigate": {"category", "region"},
    "detect": {"region"},
    "reobserve": {"region"},
    "pause": set(),
    "stop": set(),
}

SIMPLE_TOOLS = {
    "state": "get_robot_state",
    "exploration_status": "get_exploration_status",
    "map": "get_map_summary",
    "pause": "pause_exploration",
    "stop": "stop_robot",
}

STATUS_TOOLS = {"explore": "get_exploration_status", "navigate": "get_navigation_status"}
INDEXED_TASKS = {"find", "navigate", "find_and_navigate", "query"}

FAILURE_HINTS = {
    "BLOCKED": "本地任务受阻，请查看障碍和导航反馈。",
    "UNREACHABLE": "板端没有找到可达目标，请选择其他对象或区域。",
    "STALE_MAP": "地图版本已变化，请重新查询语义地图并生成新目标。",
    "LOCALIZATION_LOST": "定位已丢失，请等待板端恢复定位。",
    "PERCEPTION_LOW": "观测置信度不足，请补充观测后再查询。",
    "TIMEOUT": "请求超时，请核实板端状态后再决定下一步。",
    "CANCELLED": "板端任务已取消。",
}


def _text(value, limit=256):
    return isinstance(value, str) and len(value) <= limit and bool(value.strip()) and value.isprintable()


class _Result(Exception):
    def __init__(self, status, message, data=None):
        super().__init__(message)
        self.status = status
        self.data = data if data is not None else {}


def _finish(status, message, data=None):
    raise _Result(status, message, data)


class _Session:
    def __init__(self, call):
        self.call = call
        self.trace = []

    def request(self, tool, arguments, *, asynchronous=False):
        # Tool names and arguments are constructed only by the workflow below.
        entry = {"tool": tool, "arguments": dict(arguments)}
        self.trace.append(entry)
        try:
            reply = self.call(tool, dict(arguments))
        except TimeoutError:
            entry["status"] = "TIMEOUT"
            _finish("TIMEOUT", "网关响应超时。请先核实板端状态，以免重复发起动作。")
        except OSError:
            entry["status"] = "OFFLINE"
            _finish("OFFLINE", "网关连接失败。请先核实板端状态，以免重复发起动作。")
        except Exception as exc:
            entry["status"] = "GATEWAY_ERROR"
            _finish("GATEWAY_ERROR", f"网关客户端异常（{type(exc).__name__}），请核实板端状态。")

        if (not isinstance(reply, dict) or not isinstance(reply.get("data"), dict)
                or not isinstance(reply.get("status"), str)
                or not re.fullmatch(r"[A-Z][A-Z0-9_]{0,47}", reply["status"])):
            entry["status"] = "INVALID_RESPONSE"
            _finish("INVALID_RESPONSE", f"{tool} 返回的状态或数据格式无效。")
        status = reply["status"]
        entry["status"] = status
        if status == "RUNNING" and not asynchronous:
            _finish("RUNNING", f"{tool} 尚未完成。", reply["data"])
        if status not in {"SUCCESS", "RUNNING"}:
            message = reply.get("message")
            hint = FAILURE_HINTS.get(status, f"{tool} 返回 {status}。")
            if _text(message, 1000):
                hint = f"{hint} {message}"
            _finish(status, hint, reply["data"])
        return status, reply["data"]

    def ready(self):
        _, data = self.request("get_robot_state", {})
        if data.get("safety_ready") is not True or data.get("localization_valid") is not True:
            _finish("NOT_READY", "网关尚未确认本地安全监督就绪且定位有效，请检查机器人状态。")

    def poll(self, kind, task_id, max_polls):
        if not _text(task_id, 128):
            _finish("INVALID_RESPONSE", "异步动作缺少有效 task_id，请在板端核实执行状态。")
        progress = {"task_id": task_id}
        for attempt in range(max_polls):
            if attempt:
                time.sleep(0.3)
            try:
                status, progress = self.request(STATUS_TOOLS[kind], {"task_id": task_id}, asynchronous=True)
            except _Result as result:
                if "task_id" in result.data and result.data["task_id"] != task_id:
                    _finish("INVALID_RESPONSE", "状态回包的 task_id 不匹配。", {"task_id": task_id})
                result.data = {**result.data, "task_id": task_id, "action_kind": kind}
                raise
            if progress.get("task_id") != task_id:
                _finish("INVALID_RESPONSE", "状态回包的 task_id 不匹配。", {"task_id": task_id})
            if status == "SUCCESS":
                return progress
        _finish("RUNNING", "板端仍在执行，请使用 task_id 继续查询进度。", {**progress, "action_kind": kind})

    def action(self, kind, tool, arguments, max_polls):
        status, data = self.request(tool, arguments, asynchronous=True)
        if status == "RUNNING":
            return self.poll(kind, data.get("task_id"), max_polls)
        return data

    def query(self, arguments):
        _, data = self.request("query_semantic_map", arguments)
        objects = data.get("objects")
        if not isinstance(objects, list) or not _text(data.get("map_version"), 128):
            _finish("INVALID_RESPONSE", "语义查询需要 objects 列表和 map_version。")
        ids = set()
        for obj in objects:
            if not isinstance(obj, dict) or not _text(obj.get("object_id"), 128) or obj["object_id"] in ids:
                _finish("INVALID_RESPONSE", "语义对象缺少有效、唯一的 object_id。")
            ids.add(obj["object_id"])
        return data


def _validated_goal(goal, object_id, map_version):
    if goal.get("map_version") != map_version:
        _finish("STALE_MAP", FAILURE_HINTS["STALE_MAP"])
    if goal.get("reachable") is not True:
        _finish("UNREACHABLE", FAILURE_HINTS["UNREACHABLE"])
    if "object_id" in goal and goal["object_id"] != object_id:
        _finish("INVALID_RESPONSE", "位姿引用的目标与已选语义对象不一致。")
    keys = ("goal_id", "frame", "timestamp", "map_version")
    if not all(_text(goal.get(key), 128) for key in keys):
        _finish("INVALID_RESPONSE", "位姿引用缺少 goal_id、frame、timestamp 或 map_version。")
    try:
        stamp = datetime.fromisoformat(goal["timestamp"].replace("Z", "+00:00"))
        if stamp.utcoffset() is None:
            raise ValueError
    except ValueError:
        _finish("INVALID_RESPONSE", "目标时间戳须为带时区的 ISO 8601 时间。")
    return {key: goal[key] for key in keys}


def _workflow(session, instruction, intent, confirm_object, max_polls):
    if not isinstance(instruction, str) or not instruction.strip():
        _finish("INVALID_ARGUMENT", "请输入任务指令。")
    task = intent.get("task") if isinstance(intent, dict) else None
    if not isinstance(task, str) or task not in TASK_FIELDS:
        _finish("INVALID_ARGUMENT", "任务类型不在允许列表内。")
    if intent.keys() - (TASK_FIELDS[task] | {"task"}):
        _finish("INVALID_ARGUMENT", "任务包含未定义的参数。")
    args = {k: v for k, v in intent.items() if k != "task"}
    if not all(_text(v) for v in args.values()) or (task in INDEXED_TASKS and "category" not in args):
        _finish("INVALID_ARGUMENT", "请提供有效的任务参数，查询对象时需要 category。")
    if task == "reobserve" and "region" not in args:
        _finish("INVALID_ARGUMENT", "重新观测需要指定 region。")
    if confirm_object is not None and (task not in {"navigate", "find", "find_and_navigate"} or not _text(confirm_object, 128)):
        _finish("INVALID_ARGUMENT", "确认对象须为查询候选中的 object_id。")

    if task in SIMPLE_TOOLS:
        _, data = session.request(SIMPLE_TOOLS[task], {})
        return "已收到网关反馈。", data
    region = {"region": args["region"]} if "region" in args else {}
    if task == "detect":
        _, data = session.request("detect_objects", region)
        return "已收到当前视野的检测结果。", data
    if task == "reobserve":
        session.ready()
        _, data = session.request("request_reobservation", region)
        return "板端报告重新观测已完成。", data
    if task == "explore":
        session.ready()
        data = session.action("explore", "start_exploration", region, max_polls)
        return "板端报告探索已完成。", data

    semantic_map = session.query(args)
    if task == "query":
        return f"查询到 {len(semantic_map['objects'])} 个对象。", semantic_map
    if task in {"find", "find_and_navigate"} and not semantic_map["objects"] and confirm_object is None:
        session.ready()
        try:
            session.action("explore", "start_exploration", region, max_polls)
        except _Result as partial:
            if "task_id" in partial.data and partial.status in {"RUNNING", "TIMEOUT", "OFFLINE"}:
                following = "navigate" if task == "find_and_navigate" else "query"
                partial.data = {
                    **partial.data,
                    "action_kind": "explore",
                    "next_task_after_success": {"task": following, **args},
                }
            raise
        semantic_map = session.query(args)
    return _complete_target(session, task, semantic_map, confirm_object, max_polls)


def _complete_target(session, task, semantic_map, confirm_object, max_polls):
    objects = semantic_map["objects"]
    if not objects:
        _finish("NOT_FOUND", "当前地图中没有匹配对象。", semantic_map)
    if confirm_object is None and len(objects) > 1:
        _finish("NEEDS_CONFIRMATION", "有多个候选，请按 object_id 选择一个。", semantic_map)
    chosen = objects[0]
    if confirm_object is not None:
        chosen = next((obj for obj in objects if obj["object_id"] == confirm_object), None)
        if chosen is None:
            _finish("NEEDS_CONFIRMATION", "上次选择的对象已不在候选列表，请重新确认。", semantic_map)
    if task == "find":
        return "已在语义地图中找到对象。", {"object": chosen, "map_version": semantic_map["map_version"]}

    session.ready()
    target = {"object_id": chosen["object_id"], "map_version": semantic_map["map_version"]}
    _, goal = session.request("generate_navigation_goal", target)
    reference = _validated_goal(goal, target["object_id"], target["map_version"])
    result = session.action("navigate", "navigate_to_pose", reference, max_polls)
    return "板端报告导航已完成。", result


def _run(call, max_polls, operation):
    session = _Session(call)
    try:
        if type(max_polls) is not int or not 0 <= max_polls <= 30:
            _finish("INVALID_ARGUMENT", "max_polls 应为 0 到 30 的整数。")
        message, data = operation(session)
        return {"status": "SUCCESS", "message": message, "data": data, "trace": session.trace}
    except _Result as result:
        return {"status": result.status, "message": str(result), "data": result.data, "trace": session.trace}


def run_task(instruction, intent: dict, call, *, confirm_object=None, max_polls=4) -> dict:
    """Execute a validated intent. call(tool, args) returns {status, data, message?}."""
    return _run(call, max_polls, lambda session: _workflow(session, instruction, intent, confirm_object, max_polls))


def poll_task(kind, task_id, call, *, max_polls=1) -> dict:
    """Resume a previously returned explore/navigate action by polling only."""
    def check(session):
        if not isinstance(kind, str) or kind not in STATUS_TOOLS or not _text(task_id, 128):
            _finish("INVALID_ARGUMENT", "请提供 explore 或 navigate 和有效 task_id。")
        data = session.poll(kind, task_id, max_polls)
        return "板端报告任务已完成。", data

    return _run(call, max_polls, check)


def run_until(instruction, intent, call, *, confirm_object=None, wait_seconds=0, poll_interval=1) -> dict:
    """Wait for asynchronous exploration/navigation and continue the original task.

    The wait budget governs status polling. Each gateway RPC uses the transport timeout.
    A zero wait budget retains run_task's short polling window.
    """
    durations = (wait_seconds, poll_interval)
    if (any(type(v) not in (int, float) or isinstance(v, float) and not math.isfinite(v) for v in durations)
            or wait_seconds < 0 or poll_interval <= 0):
        return {"status": "INVALID_ARGUMENT", "message": "等待时长须为非负有限秒数，轮询间隔须为正有限秒数。",
                "data": {}, "trace": []}
    if wait_seconds == 0:
        return run_task(instruction, intent, call, confirm_object=confirm_object)

    started = time.monotonic()
    result = run_task(instruction, intent, call, confirm_object=confirm_object, max_polls=1)
    trace = list(result["trace"])
    while result["status"] == "RUNNING":
        # Only a ticket from our own exploration/navigation workflow is resumable.
        if intent["task"] not in {"find", "find_and_navigate", "explore", "navigate"}:
            break
        last_tool = trace[-1]["tool"]
        kind = result["data"].get("action_kind")
        ticket = result["data"].get("task_id")
        if kind not in ("explore", "navigate") or last_tool != STATUS_TOOLS[kind] or not _text(ticket, 128):
            break
        if time.monotonic() - started + poll_interval >= wait_seconds:
            break
        time.sleep(poll_interval)
        if time.monotonic() - started >= wait_seconds:
            break
        follow = None
        if kind == "explore" and intent["task"] in {"find", "find_and_navigate"}:
            # Derive the continuation from the validated user intent, never a gateway payload.
            follow = {key: intent[key] for key in ("category", "region") if key in intent}
        result = poll_task(kind, ticket, call)
        trace.extend(result["trace"])
        if follow is None:
            continue
        if result["status"] == "SUCCESS":
            def finish_search(session):
                return _complete_target(session, intent["task"], session.query(follow), confirm_object, 1)

            result = _run(call, 1, finish_search)
            trace.extend(result["trace"])
        elif result["status"] in {"RUNNING", "TIMEOUT", "OFFLINE"}:
            following = "query" if intent["task"] == "find" else "navigate"
            result["data"].update(action_kind=kind, next_task_after_success={"task": following, **follow})
    return {**result, "trace": trace}
