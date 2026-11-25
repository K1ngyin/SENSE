import http.client
import json
import time
import unittest
from concurrent.futures import ThreadPoolExecutor

from cloud_agent.transport import MAX_BODY, serve_gateway


class TransportTests(unittest.TestCase):
    def start_endpoint(self, timeout=2):
        endpoint = serve_gateway("test-token", port=0, timeout=timeout)
        self.broker = endpoint.__enter__()
        self.addCleanup(endpoint.__exit__, None, None, None)
        self.workers = ThreadPoolExecutor(max_workers=2)
        self.addCleanup(self.workers.shutdown)

    def post(self, path, message, *, auth="Bearer test-token", raw=False):
        body = message if raw else json.dumps(message).encode()
        client = http.client.HTTPConnection(*self.broker.address, timeout=2)
        try:
            client.request("POST", "/v1/" + path, body, {
                "Authorization": auth, "Content-Type": "application/json",
            })
            response = client.getresponse()
            return response.status, json.loads(response.read())
        finally:
            client.close()

    def command(self):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            status, body = self.post("poll", {})
            self.assertEqual(status, 200)
            if body["command"] is not None:
                return body["command"]
            time.sleep(0.005)
        self.fail("broker did not offer a command")

    def test_round_trip_and_duplicate_feedback(self):
        self.start_endpoint()
        result = self.workers.submit(self.broker.call, "query_semantic_map", {"category": "桌子"})
        command = self.command()
        self.assertEqual(command["tool"], "query_semantic_map")
        self.assertEqual(command["arguments"], {"category": "桌子"})
        self.assertIn("+00:00", command["expires_at"])
        self.assertEqual(self.post("poll", {}), (200, {"command": None}))
        reply = {"request_id": command["request_id"], "status": "SUCCESS", "data": {"n": 1}}
        self.assertEqual(self.post("result", reply), (200, {"accepted": True}))
        self.assertEqual(result.result(), {"status": "SUCCESS", "data": {"n": 1}})
        self.assertEqual(self.post("result", reply)[0], 200)
        self.assertEqual(self.post("result", {**reply, "data": {"n": True}})[0], 409)

    def test_expiry_without_delivery(self):
        self.start_endpoint(timeout=0.05)
        result = self.broker.call("get_robot_state", {})
        self.assertEqual(result["status"], "TIMEOUT")
        self.assertFalse(result["data"]["execution_unknown"])
        self.assertEqual(self.post("poll", {}), (200, {"command": None}))

    def test_lost_ack_and_late_result(self):
        self.start_endpoint(timeout=0.5)
        result = self.workers.submit(self.broker.call, "start_exploration", {"region": "lab"})
        command = self.command()
        self.assertEqual(self.post("poll", {}), (200, {"command": None}))
        timed_out = result.result(timeout=2)
        self.assertEqual(timed_out["status"], "TIMEOUT")
        self.assertTrue(timed_out["data"]["execution_unknown"])
        late = {"request_id": command["request_id"], "status": "SUCCESS", "data": {}}
        self.assertEqual(self.post("result", late)[0], 410)
        self.assertEqual(self.post("poll", {}), (200, {"command": None}))

    def test_stop_takes_priority_over_in_flight_call(self):
        self.start_endpoint()
        explore = self.workers.submit(self.broker.call, "start_exploration", {})
        self.command()
        self.assertEqual(self.broker.call("get_map_summary", {})["status"], "BLOCKED")
        stop = self.workers.submit(self.broker.call, "stop_robot", {})
        self.assertEqual(explore.result(timeout=1)["status"], "CANCELLED")
        command = self.command()
        self.assertEqual(command["tool"], "stop_robot")
        self.post("result", {"request_id": command["request_id"], "status": "SUCCESS", "data": {}})
        self.assertEqual(stop.result(timeout=1)["status"], "SUCCESS")

    def test_request_validation(self):
        self.start_endpoint()
        self.assertEqual(self.post("poll", {}, auth="Bearer wrong")[0], 401)
        self.assertEqual(self.post("result", [1])[0], 400)
        self.assertEqual(self.post("poll", {"other": 1})[0], 400)
        self.assertEqual(self.post("result", b'{"x":1,"x":2}', raw=True)[0], 400)
        self.assertEqual(self.post("result", b'{"n":1e1000}', raw=True)[0], 400)
        self.assertEqual(self.post("poll", b"a" * (MAX_BODY + 1), raw=True)[0], 413)
        self.assertEqual(self.post("other", {})[0], 404)
        with self.assertRaises(ValueError):
            self.broker.call("get_map_summary", {"x": "a" * MAX_BODY})

    def test_plaintext_binding_and_token(self):
        for args in ({"token": "ok", "host": "0.0.0.0"}, {"token": "ok", "certfile": "x"},
                     {"token": ""}, {"token": "has spaces"}, {"token": "ok", "timeout": float("nan")}):
            with self.subTest(args=args), self.assertRaises(ValueError):
                with serve_gateway(**args, port=0):
                    self.fail("invalid endpoint configuration accepted")


if __name__ == "__main__":
    unittest.main()
