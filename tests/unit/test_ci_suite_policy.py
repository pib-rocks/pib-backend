"""The unit job runs every unit test. Exclusions have to stay gone."""

from __future__ import annotations

import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
WORKFLOW = REPO_ROOT / ".github" / "workflows" / "tests.yml"
UNIT_RECIPE = REPO_ROOT / "tests" / "run_unit_suite.sh"
FULL_RUNNER = REPO_ROOT / "tests" / "run_all_tests.sh"
UNIT_DIR = REPO_ROOT / "tests" / "unit"


def _command_text(path: Path) -> str:
    """Only the command lines, never the comments.

    A comment may say "no --deselect" while describing the rule; grepping the raw
    file makes that comment fail the check that it documents.
    """
    return "\n".join(
        line
        for line in path.read_text(encoding="utf-8").splitlines()
        if not line.lstrip().startswith("#")
    )


def test_unit_workflow_has_no_ignore_or_deselect():
    text = _command_text(WORKFLOW)
    assert "--ignore" not in text
    assert "--deselect" not in text


def test_unit_job_calls_the_canonical_recipe():
    """The unit job must call tests/run_unit_suite.sh, not its own pytest line."""
    text = WORKFLOW.read_text(encoding="utf-8")
    assert "tests/run_unit_suite.sh" in text
    assert "python -m pytest" not in text
    assert "pytest tests/unit" not in text


def test_unit_recipe_does_not_exclude_tests():
    text = _command_text(UNIT_RECIPE)
    assert "python -m pytest tests/unit -q" in text
    assert "--ignore" not in text
    assert "--deselect" not in text


def test_full_runner_keeps_only_the_blockly_ignore_and_the_docker_marker():
    """tests/blockly_generator is a Node project. -m "not docker" is a marker tier."""
    text = _command_text(FULL_RUNNER)
    assert "--deselect" not in text
    ignore_lines = [line for line in text.splitlines() if "--ignore" in line]
    assert len(ignore_lines) == 1
    assert "blockly_generator" in ignore_lines[0]
    assert '-m "not docker"' in text


def test_unit_files_do_not_importorskip_at_module_level():
    offenders = []
    for path in sorted(UNIT_DIR.rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in _module_level_importorskips(tree):
            relative = path.relative_to(REPO_ROOT)
            offenders.append(f"{relative}:{node.lineno}")
    assert offenders == []


def _module_level_importorskips(tree: ast.AST) -> list[ast.Call]:
    found: list[ast.Call] = []

    def visit(node: ast.AST, nested: bool) -> None:
        if isinstance(
            node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)
        ):
            nested = True
        if not nested and isinstance(node, ast.Call) and _is_importorskip(node):
            found.append(node)
        for child in ast.iter_child_nodes(node):
            visit(child, nested)

    visit(tree, False)
    return found


def _is_importorskip(node: ast.Call) -> bool:
    func = node.func
    if isinstance(func, ast.Attribute) and func.attr == "importorskip":
        return True
    return isinstance(func, ast.Name) and func.id == "importorskip"
