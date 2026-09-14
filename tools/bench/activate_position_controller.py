#!/usr/bin/env python3
"""Activate one inactive controller through bounded, typed ROS services."""

import argparse
import os
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node

RETRY_INTERVAL_S = 0.2
READBACK_RESERVE_S = 0.2


def remaining(deadline):
    return max(0.0, deadline - time.monotonic())


def wait_for_service(client, deadline):
    while remaining(deadline) > 0.0:
        if client.wait_for_service(timeout_sec=min(0.05, remaining(deadline))):
            return True
    return False


def await_response(node, future, deadline):
    while rclpy.ok() and not future.done() and remaining(deadline) > 0.0:
        rclpy.spin_once(node, timeout_sec=min(0.02, remaining(deadline)))
    if not future.done():
        return None
    try:
        return future.result()
    except Exception:
        return None


def controller_state(node, client, controller, deadline):
    if not wait_for_service(client, deadline):
        return None
    response = await_response(node, client.call_async(ListControllers.Request()), deadline)
    if response is None:
        return None
    for item in response.controller:
        if item.name == controller:
            return item.state
    return ""


def duration_from_seconds(seconds):
    nanoseconds = max(0, int(seconds * 1_000_000_000))
    return Duration(sec=nanoseconds // 1_000_000_000,
                    nanosec=nanoseconds % 1_000_000_000)


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("controller")
    parser.add_argument("timeout_s", type=int)
    parser.add_argument("manager")
    args = parser.parse_args()

    deadline = time.monotonic() + args.timeout_s
    rclpy.init(args=None)
    node = Node(f"activate_position_controller_{os.getpid()}")
    prefix = args.manager.rstrip("/")
    list_client = node.create_client(ListControllers, f"{prefix}/list_controllers")
    switch_client = node.create_client(SwitchController, f"{prefix}/switch_controller")
    try:
        initial_state = controller_state(node, list_client, args.controller, deadline)
        if initial_state is None:
            print(f"ERROR: could not read controllers from {args.manager} before the deadline", file=sys.stderr)
            return 3
        if initial_state == "":
            print(f"ERROR: {args.controller} is not loaded on {args.manager}.", file=sys.stderr)
            print("       Load it first with the spawner's --inactive argument.", file=sys.stderr)
            return 3
        if initial_state != "inactive":
            print(f"ERROR: {args.controller} is '{initial_state}', not 'inactive'.", file=sys.stderr)
            print("       Refusing rather than reporting a success this run did not cause.", file=sys.stderr)
            return 3

        print(f"Activating {args.controller} (up to {args.timeout_s}s; refusals are expected")
        print("until feedback is flowing)...")

        attempts = 0
        reported_success = False
        ambiguous_request = False
        activation_deadline = max(time.monotonic(), deadline - READBACK_RESERVE_S)
        while remaining(activation_deadline) > 0.0:
            if not wait_for_service(switch_client, activation_deadline):
                break
            attempts += 1
            request = SwitchController.Request()
            request.activate_controllers = [args.controller]
            request.deactivate_controllers = []
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True
            request.timeout = duration_from_seconds(remaining(activation_deadline))
            response = await_response(node, switch_client.call_async(request), activation_deadline)
            if response is None:
                ambiguous_request = True
                break
            if response.ok:
                reported_success = True
                break
            delay = min(RETRY_INTERVAL_S, remaining(activation_deadline))
            if delay > 0.0:
                time.sleep(delay)

        final_state = controller_state(node, list_client, args.controller, deadline)
        state_text = "unknown" if final_state is None else (final_state or "not loaded")

        if reported_success:
            if final_state == "active":
                print(f"{args.controller} is active (confirmed by read-back, {attempts} attempts).")
                return 0
            print(f"ERROR: activation reported success but {args.controller} reads back as '{state_text}'.", file=sys.stderr)
            return 5

        if ambiguous_request:
            print(f"ERROR: the activation request did not answer before the {args.timeout_s}s total deadline;", file=sys.stderr)
            print(f"       its outcome is uncertain (read-back: '{state_text}').", file=sys.stderr)
            print("       The request may still complete after this tool exits; inspect controller_manager", file=sys.stderr)
            print("       state and logs before taking another action.", file=sys.stderr)
            return 4

        if final_state is None:
            print(f"ERROR: activation was not confirmed before the {args.timeout_s}s total deadline;", file=sys.stderr)
            print("       final controller state is unknown.", file=sys.stderr)
        else:
            print(f"ERROR: {args.controller} was refused through the {args.timeout_s}s total deadline", file=sys.stderr)
            print(f"       ({attempts} attempts); read-back confirmed '{state_text}'.", file=sys.stderr)
        print("       Check controller_manager logs and whether usable feedback is flowing.", file=sys.stderr)
        return 4
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
