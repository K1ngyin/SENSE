import json
import os
import unittest
from unittest.mock import patch

from cloud_agent.intent import interpret, parse_reply, validate


def response(args):
    func = {"name": "select_task", "arguments": json.dumps(args)}
    message = {"tool_calls": [{"type": "function", "function": func}]}
    return {"choices": [{"finish_reason": "tool_calls", "message": message}]}


class IntentTests(unittest.TestCase):
    def test_function_envelope(self):
        intent = {"task": "find", "category": "会议桌", "region": "会议室"}
        self.assertEqual(parse_reply(response(intent)), intent)

    def test_bad_model_results(self):
        invalid = [{}, response({"task": "navigate"}), response({"task": "state", "speed": 1}),
                   response({"task": "unregistered"}), response({"task": "explore", "region": 4})]
        for item in invalid:
            with self.subTest(item=item), self.assertRaises(ValueError):
                parse_reply(item)

    def test_remote_model_requires_tls(self):
        env = {"SENSE_MODEL_URL": "http://example.com/v1/chat/completions", "SENSE_MODEL": "test"}
        with patch.dict(os.environ, env), self.assertRaises(ValueError):
            interpret("查一下状态")

    def test_clarify(self):
        self.assertEqual(validate({"task": "clarify"}), {"task": "clarify"})
