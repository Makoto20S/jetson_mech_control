#!/usr/bin/env python3
"""Test-only controller manager service with deterministic activation outcomes."""
import argparse
import json
import threading
import time

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default


class TestControllerManager(Node):
    def __init__(self, args):
        super().__init__("test_activation_controller_manager")
        self.args = args
        self.state = "active" if args.scenario == "active" else "inactive"
        self.switch_count = 0
        self.lock = threading.Lock()
        prefix = args.manager.rstrip("/")
        self.create_service(ListControllers, f"{prefix}/list_controllers", self.list_controllers, qos_profile=qos_profile_services_default)
        self.create_service(SwitchController, f"{prefix}/switch_controller", self.switch_controller, qos_profile=qos_profile_services_default)

    def record(self, row):
        with self.lock, open(self.args.record, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(row) + "\n")

    def list_controllers(self, _request, response):
        self.record({"service": "list"})
        if self.args.scenario != "not_loaded":
            item = ControllerState()
            item.name = self.args.controller
            item.state = self.state
            item.type = "mech_controllers/DemoController"
            response.controller = [item]
        return response

    def switch_controller(self, request, response):
        with self.lock:
            self.switch_count += 1
            attempt = self.switch_count
        self.record({"service": "switch", "strictness": request.strictness,
                     "activate_controllers": list(request.activate_controllers),
                     "deactivate_controllers": list(request.deactivate_controllers),
                     "timeout_sec": request.timeout.sec, "timeout_nanosec": request.timeout.nanosec})
        if self.args.scenario == "hang":
            time.sleep(2.0)
            response.ok = False
        elif self.args.scenario == "refuse_forever":
            response.ok = False
        elif self.args.scenario == "refuse_twice" and attempt <= 2:
            response.ok = False
        else:
            response.ok = True
            if self.args.scenario != "lie":
                self.state = "active"
        return response


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manager", required=True)
    parser.add_argument("--controller", required=True)
    parser.add_argument("--scenario", choices=("not_loaded", "active", "refuse_twice", "refuse_forever", "lie", "hang"), required=True)
    parser.add_argument("--record", required=True)
    parser.add_argument("--ready", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = TestControllerManager(args)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    open(args.ready, "w", encoding="utf-8").close()
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
