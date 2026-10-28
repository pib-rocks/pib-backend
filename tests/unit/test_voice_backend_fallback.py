"""The duplex start-up failure must never take the node - and text chat - down.

Measured on the robot: ``build_duplex_voice_loop`` raised ``OSError`` (speaker), the error
handler called ``traceback.format_exc()`` while the name was not imported, the resulting
``NameError`` escaped ``__init__``, the process died and ``/send_chat_message`` disappeared
with it. The Cerebra UI then reported "The chat service is unavailable" for a *text* chat
that never needed the speaker.
"""

from __future__ import annotations

import ast
import builtins
import symtable
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "ros_packages/voice_assistant/voice_assistant/assistant.py"

# Modules the handler may rely on. They are only available if the source imports them, so a
# missing module-level ``import traceback`` surfaces as the NameError it really is.
STDLIB_MODULES = {
    "os",
    "time",
    "threading",
    "collections",
    "typing",
    "traceback",
    "logging",
}


def _module_level_bindings(top: "symtable.SymbolTable") -> set[str]:
    return {
        symbol.get_name()
        for symbol in top.get_symbols()
        if symbol.is_imported() or symbol.is_assigned() or symbol.is_namespace()
    } | set(dir(builtins))


def _unbound_globals(top: "symtable.SymbolTable") -> list[tuple[str, str]]:
    """(scope, name) for every global read that the module never binds.

    Scope-aware on purpose: a local ``import traceback`` inside one function does not make
    the name available to another function's handler - which is exactly the bug that killed
    the node.
    """
    bound = _module_level_bindings(top)
    missing: list[tuple[str, str]] = []

    def walk(table: "symtable.SymbolTable") -> None:
        for symbol in table.get_symbols():
            if (
                symbol.is_global()
                and symbol.is_referenced()
                and symbol.get_name() not in bound
            ):
                missing.append((table.get_name(), symbol.get_name()))
        for child in table.get_children():
            walk(child)

    walk(top)
    return missing


def test_no_scope_uses_an_unbound_global() -> None:
    """No function may read a name the module never binds at module level.

    The failure that killed the node: the handler in ``_build_duplex_loop`` used
    ``traceback`` while the only import lived inside an unrelated function, so the handler
    raised ``NameError`` instead of reporting the audio failure.
    """
    source = SOURCE.read_text(encoding="utf-8")
    top = symtable.symtable(source, str(SOURCE), "exec")
    assert _unbound_globals(top) == []


def _global_references(node: ast.AST) -> set[str]:
    """Every name loaded anywhere below ``node`` (an over-approximation is fine here)."""
    return {
        child.id
        for child in ast.walk(node)
        if isinstance(child, ast.Name) and isinstance(child.ctx, ast.Load)
    }


def _load_node_class() -> type:
    """AST-load ``VoiceAssistantNode`` with the source's own stdlib imports.

    Only the class body is compiled, so ROS, native audio and the generated datatypes never
    have to be importable. Everything the class body needs but does not import is stubbed;
    the stdlib modules above are deliberately *not* stubbed, so the handler can only use
    them when the source imports them.
    """
    source = SOURCE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    klass = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "VoiceAssistantNode"
    )
    namespace: dict[str, object] = {"__builtins__": builtins}
    for node in tree.body:
        if not isinstance(node, (ast.Import, ast.ImportFrom)):
            continue
        module = (getattr(node, "module", "") or "").split(".")[0]
        targets = {(alias.asname or alias.name).split(".")[0] for alias in node.names}
        if module in STDLIB_MODULES or targets & STDLIB_MODULES:
            exec(
                compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), "exec"),
                namespace,
            )
    for name in _global_references(klass):
        if (
            name not in namespace
            and name not in STDLIB_MODULES
            and not hasattr(builtins, name)
        ):
            # A real (empty) class, not a MagicMock: the class statement uses ``Node`` as a
            # base, and a mock base would replace the class itself.
            namespace[name] = type(name, (), {})
    body: list[ast.stmt] = [klass]
    module_ast = ast.Module(body=body, type_ignores=[])
    ast.fix_missing_locations(module_ast)
    exec(compile(module_ast, str(SOURCE), "exec"), namespace)
    return namespace["VoiceAssistantNode"]  # type: ignore[return-value]


@pytest.fixture
def failing_duplex_backend(monkeypatch):
    """A duplex backend whose speaker cannot be opened - the measured robot failure."""
    fake = types.ModuleType("voice_assistant.ros_duplex")

    def build_duplex_voice_loop(*args, **kwargs):
        raise OSError(13, "Sample format not supported")

    fake.build_duplex_voice_loop = build_duplex_voice_loop
    monkeypatch.setitem(sys.modules, "voice_assistant.ros_duplex", fake)
    return fake


def test_startup_speaker_failure_keeps_the_node_on_the_gemini_loop(
    failing_duplex_backend,
) -> None:
    """The handler must report the audio failure and return, not raise."""
    klass = _load_node_class()
    node = klass.__new__(klass)
    sentinel = object()
    node.gemini_loop = sentinel
    node.get_logger = MagicMock(return_value=MagicMock())

    loop = node._build_duplex_loop()

    assert loop is sentinel, "the node must stay on the Gemini live path"
    logger = node.get_logger.return_value
    assert logger.error.called, "the audio failure must be logged"
    message = " ".join(str(call.args[0]) for call in logger.error.call_args_list)
    assert "duplex voice backend unavailable" in message
    assert "Sample format not supported" in message
    assert "Traceback" in message, "the handler promises the traceback text"
