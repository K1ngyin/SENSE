"""In-memory teaching fixture. No hardware, credentials, networking or ROS."""

from datetime import datetime, timezone


class DemoGateway:
    def __init__(self):
        self.explored = False
        self.exploring = False
        self.navigating = False
        self.region = None

    def call(self, tool, args):
        data = {}
        status = "SUCCESS"
        if tool == "get_robot_state":
            data = {"safety_ready": True, "localization_valid": True, "map_version": "demo-map"}
        elif tool == "get_map_summary":
            data = {"map_version": "demo-map", "coverage": 0.9 if self.explored else 0.15}
        elif tool in ("detect_objects", "request_reobservation"):
            data = {"region": args.get("region"), "source": "demo", "objects": []}
        elif tool == "query_semantic_map":
            table = {"object_id": "table_01", "category": "会议桌", "region": self.region or "会议室"}
            matches = self.explored and args["category"] in ("会议桌", "table") and args.get("region", table["region"]) == table["region"]
            data = {"objects": [table] if matches else [], "map_version": "demo-map"}
        elif tool == "start_exploration":
            self.region = args.get("region")
            self.exploring = True
            status, data = "RUNNING", {"task_id": "demo-explore"}
        elif tool == "get_exploration_status":
            if self.exploring:
                self.explored, self.exploring = True, False
            data = {"task_id": "demo-explore", "coverage": 0.9 if self.explored else 0.15}
        elif tool == "generate_navigation_goal" and self.explored and args.get("object_id") == "table_01":
            data = {"goal_id": "demo-goal", "frame": "map", "map_version": "demo-map",
                    "reachable": True, "timestamp": datetime.now(timezone.utc).isoformat()}
        elif tool == "navigate_to_pose" and args.get("goal_id") == "demo-goal":
            self.navigating = True
            status, data = "RUNNING", {"task_id": "demo-nav"}
        elif tool == "get_navigation_status" and self.navigating:
            self.navigating = False
            data = {"task_id": "demo-nav", "arrived": True}
        elif tool in ("pause_exploration", "stop_robot"):
            self.exploring = False
            if tool == "stop_robot":
                self.navigating = False
        else:
            status = "UNREACHABLE"
        return {"status": status, "data": data}
