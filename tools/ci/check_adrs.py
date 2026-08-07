#!/usr/bin/env python3
"""Validate the FND-004 ADR baseline and its local Markdown links."""

from __future__ import annotations

import re
import subprocess
import sys
from datetime import date
from pathlib import Path
from urllib.parse import unquote


ADR_FILES = {
    "ADR-001": ("ADR-001-core-boundary.md", "Accepted"),
    "ADR-002": ("ADR-002-bus-runtime-ownership.md", "Accepted"),
    "ADR-003": ("ADR-003-composite-system-interface.md", "Accepted"),
    "ADR-004": ("ADR-004-fixed-protocol-profile.md", "Accepted"),
    "ADR-005": ("ADR-005-monotonic-time-freshness.md", "Accepted"),
    "ADR-006": ("ADR-006-conditional-can0-deployment.md", "Proposed"),
    "ADR-009": ("ADR-009-effort-semantic-gate.md", "Accepted"),
}

REQUIRED_HEADINGS = (
    "## Status rationale / 状态依据",
    "## Context / 上下文",
    "## Decision / 决策",
    "## Alternatives considered / 替代方案",
    "## Consequences / 后果",
    "### Positive / 正面",
    "### Negative / 负面与代价",
    "## Validation / 验证",
    "## Review triggers / 重审触发",
    "## Sources / 来源",
)

PLANNING_FILES = (
    "docs/planning/README.md",
    "docs/planning/02_architecture_and_interfaces.md",
    "docs/planning/03_mvp_delivery_plan.md",
    "docs/planning/05_decisions_and_open_questions.md",
    "docs/planning/07_framework_bootstrap_plan.md",
)

ENTRY_FILES = (
    "README.md",
    "CONTRIBUTING.md",
    "docs/development/jetson_arm64_smoke_test.md",
)

LINK_ONLY_FILES = (
    "docs/archive/README.md",
    "docs/archive/codex_ultra_master_planning_prompt.md",
    "docs/planning/01_evidence_and_research.md",
    "docs/planning/04_source_register.md",
    "docs/planning/06_cubemars_material_review.md",
)

LINK_PATTERN = re.compile(r"(?<!!)\[[^\]]+\]\(([^)]+)\)")


def fail(message: str) -> None:
    raise RuntimeError(message)


def read_text(root: Path, relative: str) -> str:
    path = root / relative
    if not path.is_file():
        fail(f"missing required file: {relative}")
    return path.read_text(encoding="utf-8")


def distributable_paths(root: Path) -> set[str]:
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
    return {
        item
        for item in result.stdout.decode("utf-8").split("\0")
        if item
    }


def validate_local_links(
    root: Path,
    relative: str,
    text: str,
    available_paths: set[str],
) -> None:
    source = root / relative
    for match in LINK_PATTERN.finditer(text):
        destination = match.group(1).strip()
        if destination.startswith("<") and destination.endswith(">"):
            destination = destination[1:-1]
        if not destination or destination.startswith(("#", "http://", "https://", "mailto:")):
            continue
        destination = unquote(destination.split("#", 1)[0].split("?", 1)[0])
        target = (source.parent / destination).resolve()
        try:
            target.relative_to(root.resolve())
        except ValueError:
            fail(f"link escapes repository: {relative} -> {destination}")
        if not target.exists():
            fail(f"broken local link: {relative} -> {destination}")
        target_relative = target.relative_to(root.resolve()).as_posix()
        if target.is_dir():
            prefix = target_relative.rstrip("/") + "/"
            is_distributable = any(path.startswith(prefix) for path in available_paths)
        else:
            is_distributable = target_relative in available_paths
        if not is_distributable:
            fail(
                "local link target is excluded from a clean clone: "
                f"{relative} -> {destination}"
            )


def metadata_value(text: str, field: str) -> str:
    prefix = f"- **{field}:** "
    values = [line.removeprefix(prefix).strip() for line in text.splitlines() if line.startswith(prefix)]
    if len(values) != 1 or not values[0]:
        fail(f"metadata field is missing or repeated: {field}")
    return values[0]


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    try:
        available_paths = distributable_paths(root)
        index_relative = "docs/adr/README.md"
        index = read_text(root, index_relative)
        if "# Architecture Decision Records / 架构决策记录" not in index:
            fail("ADR index title is missing")
        if "## FND-004 verification / FND-004 验证" not in index:
            fail("ADR index does not define the FND-004 verification entry point")

        expected_names = {filename for filename, _ in ADR_FILES.values()} | {"README.md"}
        actual_names = {path.name for path in (root / "docs/adr").glob("*.md")}
        if actual_names != expected_names:
            fail(
                "ADR file set differs: "
                f"expected={sorted(expected_names)} actual={sorted(actual_names)}"
            )

        for decision_id, (filename, expected_status) in ADR_FILES.items():
            relative = f"docs/adr/{filename}"
            text = read_text(root, relative)
            if not re.search(rf"^# {re.escape(decision_id)}：", text, re.MULTILINE):
                fail(f"{decision_id}: title or ID is missing")
            if metadata_value(text, "Decision ID") != decision_id:
                fail(f"{decision_id}: metadata Decision ID differs")
            if metadata_value(text, "Status") != expected_status:
                fail(f"{decision_id}: expected status {expected_status}")
            try:
                date.fromisoformat(metadata_value(text, "Date"))
            except ValueError:
                fail(f"{decision_id}: date is missing or invalid")
            metadata_value(text, "Owner")
            metadata_value(text, "Scope")
            for heading in REQUIRED_HEADINGS:
                if heading not in text:
                    fail(f"{decision_id}: missing heading: {heading}")
            for marker in ("[TODO]", "TBD", "待补充"):
                if marker in text:
                    fail(f"{decision_id}: unresolved placeholder: {marker}")
            if f"[{decision_id}]({filename})" not in index:
                fail(f"{decision_id}: ADR index link is missing")
            if not re.search(
                rf"\| \[{re.escape(decision_id)}\]\({re.escape(filename)}\) "
                rf"\| {expected_status} \|",
                index,
            ):
                fail(f"{decision_id}: ADR index status differs")
            validate_local_links(root, relative, text, available_paths)

        validate_local_links(root, index_relative, index, available_paths)

        planning_texts = {
            relative: read_text(root, relative) for relative in PLANNING_FILES
        }
        for relative, text in planning_texts.items():
            if "../adr/README.md" not in text:
                fail(f"planning reverse link to ADR index is missing: {relative}")
            validate_local_links(root, relative, text, available_paths)

        for relative in ENTRY_FILES:
            text = read_text(root, relative)
            if "docs/adr/README.md" not in text and "../adr/README.md" not in text:
                fail(f"ADR index entry link is missing: {relative}")
            validate_local_links(root, relative, text, available_paths)

        for relative in LINK_ONLY_FILES:
            validate_local_links(
                root,
                relative,
                read_text(root, relative),
                available_paths,
            )

        for decision_id, (filename, _) in ADR_FILES.items():
            direct_link = f"../adr/{filename}"
            for relative in (
                "docs/planning/02_architecture_and_interfaces.md",
                "docs/planning/07_framework_bootstrap_plan.md",
            ):
                if direct_link not in planning_texts[relative]:
                    fail(f"{decision_id}: direct planning reverse link missing in {relative}")
                expected_status = ADR_FILES[decision_id][1]
                expected_row = f"| [{decision_id}]({direct_link}) | {expected_status} |"
                if expected_row not in planning_texts[relative]:
                    fail(
                        f"{decision_id}: planning status differs from {expected_status} "
                        f"in {relative}"
                    )

        planning_index = planning_texts["docs/planning/README.md"]
        foundation_plan = planning_texts["docs/planning/07_framework_bootstrap_plan.md"]
        for text, relative in (
            (planning_index, "docs/planning/README.md"),
            (foundation_plan, "docs/planning/07_framework_bootstrap_plan.md"),
        ):
            if "FND-004 已完成" not in text or "FND-004A" not in text:
                fail(f"FND-004 completion/next gate is not recorded in {relative}")

    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1

    accepted = sum(status == "Accepted" for _, status in ADR_FILES.values())
    proposed = sum(status == "Proposed" for _, status in ADR_FILES.values())
    print(
        "PASS: FND-004 ADR baseline validated "
        f"({len(ADR_FILES)} decisions: {accepted} Accepted, {proposed} Proposed)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
