"""Turn one utterance into a constrained high-level task using Chat Completions."""

import json
import os
from urllib import error, parse, request


TASKS = ("state", "exploration_status", "map", "query", "explore", "find",
         "navigate", "find_and_navigate", "detect", "reobserve", "pause", "stop", "clarify")
FUNCTION = {
    "name": "select_task",
    "description": "Select one high-level task explicitly requested by the user.",
    "parameters": {
        "type": "object",
        "properties": {
            "task": {"type": "string", "enum": list(TASKS)},
            "category": {"type": "string", "description": "Object category in the team's vocabulary"},
            "region": {"type": "string", "description": "Named area if specified"},
        },
        "required": ["task"],
        "additionalProperties": False,
    },
}
PROMPT = """你是室内机器人任务分类器。用 select_task 返回一个意图。
查询机器人状态 state，覆盖率 exploration_status，地图摘要 map，查询已有物体 query，
探索区域 explore，探索或寻找物体 find，前往已知物体附近 navigate，明确要求寻找并前往 find_and_navigate，
识别当前视野 detect，重新观察指定区域 reobserve，
暂停探索 pause，停止小车 stop。不清楚、条件无法表达、底盘速度或其他请求选 clarify。
纯查询不得选择移动。保留限定区域和物体类别，不猜坐标、物体编号或选择结果。
category 和 region 使用对接词表；无约定时保留用户原词。"""


def validate(value):
    if not isinstance(value, dict) or set(value) - {"task", "category", "region"}:
        raise ValueError("意图字段无效")
    task = value.get("task")
    if task not in TASKS:
        raise ValueError("未知任务类型")
    allowed = {"task"}
    if task in ("query", "find", "navigate", "find_and_navigate"):
        allowed.add("category")
        if not value.get("category"):
            raise ValueError("请提供要查询或前往的物体类别")
    if task in ("explore", "query", "find", "navigate", "find_and_navigate", "detect", "reobserve"):
        allowed.add("region")
    if task == "reobserve" and not value.get("region"):
        raise ValueError("请提供要重新观察的区域")
    if set(value) - allowed:
        raise ValueError("任务包含未支持的限定条件")
    if any(not isinstance(v, str) or not v.strip() or len(v) > 120 for v in value.values()):
        raise ValueError("意图参数必须为非空短文本")
    return {k: v.strip() for k, v in value.items()}


class NoRedirect(request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def parse_reply(body):
    try:
        choice = body["choices"][0]
        calls = choice["message"]["tool_calls"]
        if choice.get("finish_reason") != "tool_calls" or len(calls) != 1:
            raise ValueError
        call = calls[0]
        if call.get("type") != "function" or call["function"]["name"] != FUNCTION["name"]:
            raise ValueError
        arguments = json.loads(call["function"]["arguments"])
    except (IndexError, KeyError, TypeError, ValueError) as exc:
        raise ValueError("模型未返回完整且唯一的 select_task 调用") from exc
    return validate(arguments)


def interpret(instruction):
    if not instruction.strip() or len(instruction) > 2000:
        raise ValueError("指令须为 1 至 2000 个字符")
    endpoint = os.environ.get("SENSE_MODEL_URL", "")
    model = os.environ.get("SENSE_MODEL", "")
    key = os.environ.get("SENSE_MODEL_KEY", "")
    url = parse.urlsplit(endpoint)
    local = url.hostname in ("127.0.0.1", "::1", "localhost")
    if not model or not url.hostname or url.username or url.password or url.query or url.fragment:
        raise ValueError("请配置 SENSE_MODEL 和完整的 SENSE_MODEL_URL（chat/completions 地址）")
    if url.scheme != "https" and not (local and url.scheme == "http"):
        raise ValueError("远程模型端点需要 HTTPS")
    headers = {"Content-Type": "application/json"}
    if key:
        headers["Authorization"] = "Bearer " + key
    data = {
        "model": model,
        "messages": [{"role": "system", "content": PROMPT}, {"role": "user", "content": instruction}],
        "tools": [{"type": "function", "function": FUNCTION}],
        "tool_choice": {"type": "function", "function": {"name": FUNCTION["name"]}},
    }
    req = request.Request(endpoint, json.dumps(data).encode(), headers)
    try:
        with request.build_opener(NoRedirect()).open(req, timeout=25) as response:
            raw = response.read(65537)
    except error.HTTPError as exc:
        raise ValueError(f"模型服务返回 HTTP {exc.code}") from exc
    except (OSError, error.URLError) as exc:
        raise ValueError("无法取得模型响应") from exc
    if len(raw) > 65536:
        raise ValueError("模型响应过大")
    return parse_reply(json.loads(raw))
