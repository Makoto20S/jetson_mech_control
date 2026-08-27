#!/usr/bin/env python3
"""Validate the portable Foundation workspace boundary without ROS tooling."""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree


EXPECTED_PACKAGES = {
    "mech_control_core": set(),
    "mech_simulation": {"mech_control_core"},
    "mech_hardware_ros2_control": {"mech_control_core"},
    "mech_controllers": {"mech_control_core"},
    "mech_bringup": {
        "mech_control_core",
        "mech_simulation",
        "mech_hardware_ros2_control",
        "mech_controllers",
    },
}

REQUIRED_FILES = {
    ".dockerignore",
    ".github/ISSUE_TEMPLATE/foundation-task.yml",
    ".github/pull_request_template.md",
    ".github/workflows/foundation.yml",
    "AGENTS.md",
    "CONTRIBUTING.md",
    "docker/ros_humble_jammy/Dockerfile",
    "docs/adr/ADR-001-core-boundary.md",
    "docs/adr/ADR-002-bus-runtime-ownership.md",
    "docs/adr/ADR-003-composite-system-interface.md",
    "docs/adr/ADR-004-fixed-protocol-profile.md",
    "docs/adr/ADR-005-monotonic-time-freshness.md",
    "docs/adr/ADR-006-conditional-can0-deployment.md",
    "docs/adr/ADR-009-effort-semantic-gate.md",
    "docs/adr/README.md",
    "docs/archive/README.md",
    "docs/archive/codex_ultra_master_planning_prompt.md",
    "docs/development/ai_collaboration_workflow.md",
    "docs/development/jetson_arm64_smoke_test.md",
    "docs/development/jetson_orin_nx_jetpack6_upgrade_guide.md",
    "docs/planning/07_framework_bootstrap_plan.md",
    "manifests/ai_skills.yaml",
    "manifests/dependencies.json",
    "manifests/dependencies.repos",
    "ros2_ws/src/mech_control_core/package.xml",
    "ros2_ws/src/mech_simulation/package.xml",
    "ros2_ws/src/mech_hardware_ros2_control/package.xml",
    "ros2_ws/src/mech_controllers/package.xml",
    "ros2_ws/src/mech_bringup/package.xml",
    "tools/ci/build_workspace.sh",
    "tools/ci/check_adrs.py",
    "tools/ci/context_check.py",
}

FORBIDDEN_TRACKED_PARTS = {
    "CubeMars",
    ".codex",
    ".agents",
    "memory",
    "tmp",
    "presentation",
    "company",
}

# Directories that are local-only/confidential and must never reach the Docker
# build context. Every name here must appear in .dockerignore (as a bare
# name so it matches the directory regardless of nesting) or a local-only
# asset can silently leak into a built image even though it is correctly
# excluded from Git by .gitignore. This list is intentionally independent of
# FORBIDDEN_TRACKED_PARTS: it is checked against .dockerignore, not Git.
DOCKER_CONTEXT_MUST_EXCLUDE = {
    "CubeMars",
    ".codex",
    ".agents",
    "memory",
    "tmp",
    "presentation",
    "company",
}

PORTABLE_FILES = {
    ".dockerignore",
    ".github/workflows/foundation.yml",
    "docker/ros_humble_jammy/Dockerfile",
    "docs/adr/ADR-001-core-boundary.md",
    "docs/adr/ADR-002-bus-runtime-ownership.md",
    "docs/adr/ADR-003-composite-system-interface.md",
    "docs/adr/ADR-004-fixed-protocol-profile.md",
    "docs/adr/ADR-005-monotonic-time-freshness.md",
    "docs/adr/ADR-006-conditional-can0-deployment.md",
    "docs/adr/ADR-009-effort-semantic-gate.md",
    "docs/adr/README.md",
    "docs/archive/README.md",
    "manifests/dependencies.json",
    "manifests/dependencies.repos",
    "tools/ci/build_workspace.sh",
    "tools/ci/check_adrs.py",
}


def fail(message: str) -> None:
    raise RuntimeError(message)


def read_text(root: Path, relative: str) -> str:
    path = root / relative
    if not path.is_file():
        fail(f"missing required file: {relative}")
    return path.read_text(encoding="utf-8")


def tracked_paths(root: Path) -> list[str]:
    result = subprocess.run(
        [
            "git",
            "-C",
            str(root),
            "ls-files",
            "-z",
            "--cached",
            "--others",
            "--exclude-standard",
        ],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return [item for item in result.stdout.decode("utf-8").split("\0") if item]


def package_dependencies(package_xml: Path) -> tuple[str, set[str]]:
    try:
        document = ElementTree.parse(package_xml).getroot()
    except ElementTree.ParseError as error:
        fail(f"invalid XML in {package_xml}: {error}")
    name = document.findtext("name")
    if not name or not re.fullmatch(r"[a-z][a-z0-9_]*", name):
        fail(f"invalid package name in {package_xml}")
    if document.get("format") != "3":
        fail(f"{name}: package.xml must use format 3")
    dependencies: set[str] = set()
    for tag in (
        "depend",
        "build_depend",
        "build_export_depend",
        "exec_depend",
        "run_depend",
        "test_depend",
    ):
        dependencies.update(
            value.text.strip()
            for value in document.findall(tag)
            if value.text and value.text.strip()
        )
    return name, dependencies


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    try:
        for relative in REQUIRED_FILES:
            if not (root / relative).is_file():
                fail(f"missing required file: {relative}")

        subprocess.run(
            [sys.executable, str(root / "tools/ci/check_adrs.py")],
            check=True,
            cwd=root,
        )

        manifest = json.loads(read_text(root, "manifests/dependencies.json"))
        if manifest.get("schema_version") != 1:
            fail("dependencies.json: unsupported schema_version")
        target = manifest.get("target", {})
        if target.get("os_release") != "22.04":
            fail("dependencies.json: target os_release must be 22.04")
        if target.get("ros_distribution") != "humble":
            fail("dependencies.json: target ROS distribution must be humble")
        if target.get("cxx_standard") != 17:
            fail("dependencies.json: C++ standard must be 17")
        if set(target.get("architectures", [])) != {"amd64", "arm64"}:
            fail("dependencies.json: expected amd64 and arm64 targets")
        rosdep = manifest.get("rosdep", {})
        if rosdep.get("default_command") != "rosdep":
            fail("dependencies.json: official rosdep must remain the default")
        if rosdep.get("command_override_environment") != "ROSDEP_COMMAND":
            fail("dependencies.json: ROSDEP_COMMAND override is missing")
        base_image = manifest.get("base_image", {})
        digest = base_image.get("digest", "")
        if not re.fullmatch(r"sha256:[0-9a-f]{64}", digest):
            fail("dependencies.json: base image digest is not immutable")
        if base_image.get("floating_tags_forbidden") is not True:
            fail("dependencies.json: floating_tags_forbidden must be true")
        for platform in ("linux/amd64", "linux/arm64"):
            if not re.fullmatch(
                r"sha256:[0-9a-f]{64}",
                base_image.get("platform_digests", {}).get(platform, ""),
            ):
                fail(f"dependencies.json: missing digest for {platform}")
        dockerfile = read_text(root, "docker/ros_humble_jammy/Dockerfile")
        for tool in manifest.get("base_image_tools", []):
            if not isinstance(tool, str) or not tool.strip():
                fail("dependencies.json: base_image_tools contains an invalid entry")
            if not re.search(rf"(?m)^\s+{re.escape(tool)}\s+\\?$", dockerfile):
                fail(f"Dockerfile is missing manifest build tool: {tool}")
        if read_text(root, "manifests/dependencies.repos").strip().splitlines()[-1] != "repositories: {}":
            fail("dependencies.repos: expected an explicit empty repository set")

        package_root = root / "ros2_ws/src"
        discovered = {
            path.parent.name: path for path in package_root.glob("*/package.xml")
        }
        if set(discovered) != set(EXPECTED_PACKAGES):
            fail(
                "workspace packages differ: "
                f"expected={sorted(EXPECTED_PACKAGES)} actual={sorted(discovered)}"
            )
        for expected_name, expected_dependencies in EXPECTED_PACKAGES.items():
            declared_name, dependencies = package_dependencies(discovered[expected_name])
            if declared_name != expected_name:
                fail(f"directory/package mismatch: {expected_name} declares {declared_name}")
            internal_dependencies = dependencies & set(EXPECTED_PACKAGES)
            if internal_dependencies != expected_dependencies:
                fail(
                    f"{expected_name}: internal dependency boundary differs: "
                    f"expected={sorted(expected_dependencies)} "
                    f"actual={sorted(internal_dependencies)}"
                )
            package_dir = discovered[expected_name].parent
            if not (package_dir / "CMakeLists.txt").is_file():
                fail(f"{expected_name}: CMakeLists.txt is missing")
            if not (package_dir / "test/test_package_marker.cpp").is_file():
                fail(f"{expected_name}: skeleton test is missing")

        expected_from = "FROM ros:humble-ros-base-jammy@" + base_image["digest"]
        if expected_from not in dockerfile:
            fail("Dockerfile base image does not match dependencies.json")
        if re.search(r"FROM\s+[^\n]*:latest\b", dockerfile):
            fail("Dockerfile must not use a latest tag")
        build_script = read_text(root, "tools/ci/build_workspace.sh")
        if 'ROSDEP_COMMAND="${ROSDEP_COMMAND:-rosdep}"' not in build_script:
            fail("build script must default ROSDEP_COMMAND to official rosdep")
        if '"${ROSDEP_COMMAND}" install' not in build_script:
            fail("build script does not use the selected dependency resolver")
        workflow = read_text(root, ".github/workflows/foundation.yml")
        if "tools/ci/context_check.py" not in workflow:
            fail("workflow does not invoke context_check.py")
        if "docker/ros_humble_jammy/Dockerfile" not in workflow:
            fail("workflow does not build the pinned Humble image")
        if not re.search(r"actions/checkout@[0-9a-f]{40}", workflow):
            fail("workflow actions must be pinned to full commit SHAs")

        agents = read_text(root, "AGENTS.md")
        skills_manifest = read_text(root, "manifests/ai_skills.yaml")
        for skill in ("project-memory", "write-codex-handoff"):
            if skill not in agents or skill not in skills_manifest:
                fail(f"approved AI skill boundary is missing: {skill}")
        if "GitHub Issues/Milestones" not in agents:
            fail("AGENTS.md must identify the shared task-status system")
        if "project_memory_git_tracking: forbidden" not in skills_manifest:
            fail("ai_skills.yaml must keep project memory local-only")

        for relative in PORTABLE_FILES:
            content = read_text(root, relative)
            for forbidden in (r"[A-Za-z]:\\", r"/home/", r"/mnt/"):
                if re.search(forbidden, content):
                    fail(f"portable file contains host-specific path: {relative}")

        for path in tracked_paths(root):
            parts = set(Path(path).parts)
            if parts & FORBIDDEN_TRACKED_PARTS:
                fail(f"forbidden path is tracked: {path}")
            if Path(path).suffix.lower() in {".pdf", ".zip", ".rar", ".exe", ".bag", ".log"}:
                fail(f"forbidden artifact is tracked: {path}")

        # Guard against the .dockerignore/.gitignore drift class of bug: every
        # local-only/confidential directory must be excluded from the Docker
        # build context, not just from Git. `COPY . /workspace` in the
        # Dockerfile means anything missing here can be baked into an image.
        dockerignore_lines = {
            line.strip()
            for line in read_text(root, ".dockerignore").splitlines()
            if line.strip() and not line.strip().startswith("#")
        }
        for excluded_name in sorted(DOCKER_CONTEXT_MUST_EXCLUDE):
            if excluded_name not in dockerignore_lines:
                fail(
                    ".dockerignore is missing a required entry: "
                    f"'{excluded_name}' (present in DOCKER_CONTEXT_MUST_EXCLUDE "
                    "but absent from .dockerignore)"
                )

    except (OSError, subprocess.SubprocessError, json.JSONDecodeError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1

    print(
        "PASS: portable context, pinned dependency manifest, "
        f"and {len(EXPECTED_PACKAGES)} ROS package skeletons validated"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
