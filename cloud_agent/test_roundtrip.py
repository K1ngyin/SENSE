import io
import json
import unittest
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import patch

from cloud_agent.agent import run_task
from cloud_agent.intent import interpret
from cloud_agent.readonly_link import read_one
from cloud_agent.transport import serve_gateway


class RoundTripTests(unittest.TestCase):
    def test_model_contract_to_readonly_gateway(self):
        # Model response is a fixture; the HTTP gateway exchange is real loopback I/O.
        envelope = {"choices": [{"finish_reason": "tool_calls", "message": {"tool_calls": [
            {"type": "function", "function": {"name": "select_task", "arguments": '{"task":"state"}'}}
        ]}}]}
        env = {"SENSE_MODEL": "fixture", "SENSE_MODEL_URL": "https://example.invalid/v1/chat/completions"}
        with patch.dict("os.environ", env), patch("cloud_agent.intent.request.build_opener") as client:
            client.return_value.open.return_value = io.BytesIO(json.dumps(envelope).encode())
            intent = interpret("板子现在是什么状态？")
            req = client.return_value.open.call_args.args[0]
            self.assertEqual(json.loads(req.data)["messages"][1]["content"], "板子现在是什么状态？")
        with serve_gateway("local-fixture", port=0, timeout=4) as broker, ThreadPoolExecutor(1) as workers:
            worker = workers.submit(read_one, "local-fixture", broker.address[1], 3)
            result = run_task("板子现在是什么状态？", intent, broker.call)
            worker.result(timeout=4)
        self.assertEqual(result["status"], "SUCCESS")
        self.assertEqual(result["data"]["source"], "readonly_host_probe")
        self.assertIsNone(result["data"]["safety_ready"])
        self.assertTrue(result["data"]["hostname"])
