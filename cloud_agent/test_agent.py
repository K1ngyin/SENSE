import unittest
from unittest.mock import Mock, patch

from cloud_agent.agent import poll_task, run_task, run_until


def ok(**data):
    return {"status": "SUCCESS", "data": data}


def running(task_id):
    return {"status": "RUNNING", "data": {"task_id": task_id}}


def objects(*ids, version="map-1"):
    return ok(map_version=version, objects=[{"object_id": object_id} for object_id in ids])


READY = ok(safety_ready=True, localization_valid=True)
GOAL = {"goal_id": "goal-17", "frame": "map", "timestamp": "2026-11-25T05:45:00Z", "map_version": "map-1"}


class WorkflowTests(unittest.TestCase):
    def run_intent(self, task, replies, *, category="table", region=None, **options):
        caller = Mock(side_effect=replies)
        intent = {"task": task}
        if task in {"query", "find", "navigate", "find_and_navigate"}:
            intent["category"] = category
        if region:
            intent["region"] = region
        result = run_task("找桌子", intent, caller, **options)
        return result, [tuple(entry.args) for entry in caller.call_args_list]

    def test_find_explores_once_then_queries_against_new_map(self):
        result, calls = self.run_intent("find", [
            objects(), READY, running("explore-2"), ok(task_id="explore-2"),
            objects("table-1", version="map-2"),
        ], region="会议室")
        self.assertEqual(result["status"], "SUCCESS")
        self.assertEqual(result["data"]["map_version"], "map-2")
        query = ("query_semantic_map", {"category": "table", "region": "会议室"})
        self.assertEqual(calls, [query, ("get_robot_state", {}),
                                 ("start_exploration", {"region": "会议室"}),
                                 ("get_exploration_status", {"task_id": "explore-2"}), query])

    def test_navigation_forwards_only_gateway_reference_and_correlates_completion(self):
        result, calls = self.run_intent("navigate", [
            objects("a", "b"), READY,
            ok(**GOAL, reachable=True, object_id="b", pose=[1, 2, 0], linear_x=0.2),
            running("nav-9"), ok(task_id="nav-9", arrived=True),
        ], confirm_object="b")
        self.assertEqual(result["status"], "SUCCESS")
        self.assertEqual(calls[2:], [
            ("generate_navigation_goal", {"object_id": "b", "map_version": "map-1"}),
            ("navigate_to_pose", GOAL), ("get_navigation_status", {"task_id": "nav-9"}),
        ])

    def test_find_and_navigate_after_exploration(self):
        result, calls = self.run_intent("find_and_navigate", [
            objects(), READY, ok(), objects("t"), READY, ok(**GOAL, reachable=True), ok(),
        ])
        self.assertEqual(result["status"], "SUCCESS")
        self.assertEqual([name for name, _ in calls], [
            "query_semantic_map", "get_robot_state", "start_exploration", "query_semantic_map",
            "get_robot_state", "generate_navigation_goal", "navigate_to_pose",
        ])

    def test_detection_and_reobservation(self):
        result, calls = self.run_intent("detect", [ok(objects=[])])
        self.assertEqual((result["status"], calls), ("SUCCESS", [("detect_objects", {})]))
        result, calls = self.run_intent("reobserve", [READY, ok()], region="hall")
        self.assertEqual((result["status"], calls), ("SUCCESS", [
            ("get_robot_state", {}), ("request_reobservation", {"region": "hall"}),
        ]))
        result, calls = self.run_intent("reobserve", [])
        self.assertEqual((result["status"], calls), ("INVALID_ARGUMENT", []))

    def test_unready_state_cannot_dispatch_motion(self):
        for data in [{}, {"safety_ready": "true", "localization_valid": True},
                     {"safety_ready": True, "localization_valid": False}]:
            with self.subTest(data=data):
                result, calls = self.run_intent("explore", [ok(**data)])
                self.assertEqual(result["status"], "NOT_READY")
                self.assertEqual(calls, [("get_robot_state", {})])

    def test_ambiguous_vanished_or_missing_target_never_moves(self):
        for candidates, confirmed, status in [
            (("a", "b"), None, "NEEDS_CONFIRMATION"),
            (("a",), "b", "NEEDS_CONFIRMATION"),
            ((), None, "NOT_FOUND"),
        ]:
            result, calls = self.run_intent("navigate", [objects(*candidates)], confirm_object=confirmed)
            self.assertEqual((result["status"], len(calls)), (status, 1))

    def test_goal_must_be_reachable_current_and_complete(self):
        for overrides, status in [
            ({"map_version": "map-2"}, "STALE_MAP"), ({"reachable": False}, "UNREACHABLE"),
            ({"reachable": "true"}, "UNREACHABLE"), ({"goal_id": ""}, "INVALID_RESPONSE"),
            ({"object_id": "other"}, "INVALID_RESPONSE"), ({"timestamp": "now"}, "INVALID_RESPONSE"),
            ({"timestamp": "2026-11-25T05:45:00"}, "INVALID_RESPONSE"),
        ]:
            result, calls = self.run_intent("navigate", [
                objects("a"), READY, ok(**{**GOAL, "reachable": True, **overrides}),
            ])
            self.assertEqual((result["status"], len(calls)), (status, 3))

    def test_running_does_not_claim_completion_or_replay(self):
        result, calls = self.run_intent("find", [objects(), READY, running("x"), running("x")],
                                        max_polls=1)
        self.assertEqual((result["status"], result["data"]["task_id"]), ("RUNNING", "x"))
        self.assertEqual(result["data"]["action_kind"], "explore")
        self.assertEqual(result["data"]["next_task_after_success"], {"task": "query", "category": "table"})
        self.assertEqual(calls[-1], ("get_exploration_status", {"task_id": "x"}))
        caller = Mock(return_value=ok(task_id="x"))
        final = poll_task("explore", "x", caller)
        self.assertEqual(final["status"], "SUCCESS")
        caller.assert_called_once_with("get_exploration_status", {"task_id": "x"})

    def test_timeout_on_submit_does_not_replay(self):
        result, calls = self.run_intent("explore", [READY, TimeoutError()])
        self.assertEqual((result["status"], len(calls)), ("TIMEOUT", 2))
        self.assertEqual(result["trace"][-1]["status"], "TIMEOUT")

    def test_poll_failure_keeps_ticket_without_replaying(self):
        for reply, status in [(TimeoutError(), "TIMEOUT"), (ok(task_id="other"), "INVALID_RESPONSE"),
                              ({"status": "BLOCKED", "data": {"reason": "obstacle"}}, "BLOCKED")]:
            result, calls = self.run_intent("explore", [READY, running("id1"), reply])
            self.assertEqual((result["status"], result["data"]["task_id"], len(calls)), (status, "id1", 3))

    def test_stop_is_immediate_and_state_query_is_read_only(self):
        for task, expected in [("stop", "stop_robot"), ("pause", "pause_exploration"),
                               ("state", "get_robot_state"), ("map", "get_map_summary"),
                               ("exploration_status", "get_exploration_status")]:
            result, calls = self.run_intent(task, [ok()])
            self.assertEqual((result["status"], calls), ("SUCCESS", [(expected, {})]))

    def test_unregistered_tools_and_unexpected_fields_never_call_gateway(self):
        for intent in [{"task": "cmd_vel"}, {"task": "navigate", "category": "table", "pose": [0, 0]},
                       {"task": "stop", "speed": 0}, {"task": "navigate", "region": "hall"},
                       {"task": "query", "category": 3}, {"task": []}]:
            caller = Mock()
            self.assertEqual(run_task("test", intent, caller)["status"], "INVALID_ARGUMENT")
            caller.assert_not_called()

    def test_malformed_response_never_drives(self):
        for reply in [None, {}, ok(), ok(objects=[{"object_id": "x"}, {"object_id": "x"}], map_version="v")]:
            result, calls = self.run_intent("navigate", [reply])
            self.assertEqual((result["status"], len(calls)), ("INVALID_RESPONSE", 1))

    def test_wait_continues_long_exploration_then_long_navigation_once(self):
        caller = Mock(side_effect=[
            objects(), READY, running("x"), *[running("x") for _ in range(5)], ok(task_id="x"),
            objects("desk"), READY, ok(**GOAL, reachable=True), running("n"), running("n"), ok(task_id="n"),
        ])
        with patch("cloud_agent.agent.time.sleep"):
            result = run_until("找到桌子并过去", {"task": "find_and_navigate", "category": "table"},
                               caller, wait_seconds=60)
        self.assertEqual(result["status"], "SUCCESS")
        self.assertEqual(result["data"]["task_id"], "n")
        used = [entry["tool"] for entry in result["trace"]]
        self.assertEqual(used.count("start_exploration"), 1)
        self.assertEqual(used.count("navigate_to_pose"), 1)
        self.assertEqual(used.count("query_semantic_map"), 2)
        self.assertEqual(used.count("get_exploration_status"), 6)
        self.assertEqual(len(used), caller.call_count)

    def test_failed_long_action_is_not_replayed(self):
        caller = Mock(side_effect=[objects(), READY, running("x"), running("x"), TimeoutError()])
        with patch("cloud_agent.agent.time.sleep"):
            result = run_until("找桌子", {"task": "find", "category": "table"}, caller, wait_seconds=60)
        self.assertEqual((result["status"], caller.call_count), ("TIMEOUT", 5))
        self.assertEqual(result["data"], {"task_id": "x", "action_kind": "explore",
                                         "next_task_after_success": {"task": "query", "category": "table"}})

    def test_after_long_search_empty_and_ambiguous_are_terminal(self):
        for candidates, expected in [((), "NOT_FOUND"), (("a", "b"), "NEEDS_CONFIRMATION")]:
            for task in ("find", "find_and_navigate"):
                caller = Mock(side_effect=[objects(), READY, running("x"), running("x"), ok(task_id="x"),
                                           objects(*candidates)])
                with patch("cloud_agent.agent.time.sleep"):
                    result = run_until("找桌子", {"task": task, "category": "table"}, caller, wait_seconds=60)
                self.assertEqual((result["status"], caller.call_count), (expected, 6))

    def test_expired_wait_returns_resume_information(self):
        caller = Mock(side_effect=[objects(), READY, running("x"), running("x"), running("x")])
        with patch("cloud_agent.agent.time.monotonic", side_effect=[0, 0, 1, 1.5]), patch("cloud_agent.agent.time.sleep"):
            result = run_until("找到桌子并过去", {"task": "find_and_navigate", "category": "table"},
                               caller, wait_seconds=2, poll_interval=1)
        self.assertEqual(result["status"], "RUNNING")
        self.assertEqual(result["data"], {"task_id": "x", "action_kind": "explore",
                                         "next_task_after_success": {"task": "navigate", "category": "table"}})
        self.assertEqual(caller.call_count, 5)


if __name__ == "__main__":
    unittest.main()
