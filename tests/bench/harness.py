"""Runs the mechanics Q&A benchmark against the MCP tools. No LLM anywhere.

The benchmark's job is not to score a model. It is to prove that the *tools* can
answer the questions people actually ask -- and that the answers do not drift
when the code underneath changes. A question here is a tool call plus the number
it must come back with, so it fails the same way a unit test does, in CI, with
no API key and no network.

The optional second half -- asking Claude Code the same questions in prose and
seeing whether it calls the tools instead of guessing -- lives in README.md,
because that part needs a human to read the transcript.
"""

from __future__ import annotations

import asyncio
import re
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, ConfigDict, Field

QUESTIONS = Path(__file__).parent / "questions.yaml"
FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"


class Expectation(BaseModel):
    """One number, and how close the tool has to get to it."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    path: str = Field(description="Where in the result the answer is, e.g. joints[joint=j2].ok")
    value: Any = Field(default=None, description="What it must be.")
    tol: float | None = Field(
        default=None,
        description="Absolute tolerance. Without it the comparison is exact, which is what "
        "you want for a verdict, a name or a list -- and never what you want for a torque.",
    )


class Question(BaseModel):
    """One benchmark entry: a question, the call that answers it, and the answer."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str
    question: str = Field(description="In the words a user would use. This is the point of it.")
    tool: str
    arguments: dict[str, Any] = Field(default_factory=dict)
    expect: list[Expectation]
    note: str | None = None


class Outcome(BaseModel):
    """What one question did when it was run."""

    model_config = ConfigDict(frozen=True)

    question: Question
    passed: bool
    failures: list[str] = Field(default_factory=list)

    @property
    def summary(self) -> str:
        return "pass" if self.passed else "; ".join(self.failures)


def load(path: Path = QUESTIONS) -> list[Question]:
    raw = yaml.safe_load(path.read_text()) or []
    questions = [Question.model_validate(entry) for entry in raw]
    ids = [q.id for q in questions]
    duplicates = {name for name in ids if ids.count(name) > 1}
    if duplicates:
        raise ValueError(f"duplicate question ids: {', '.join(sorted(duplicates))}")
    return questions


def call(tool: str, arguments: dict[str, Any]) -> dict[str, Any]:
    """One MCP tool call, with ``{dast1}`` in any argument pointing at the fixture."""
    from mechlint.mcp import server

    filled = {
        name: value.format(dast1=FIXTURES / "dast1") if isinstance(value, str) else value
        for name, value in arguments.items()
    }
    result = asyncio.run(server.server.call_tool(tool, filled))
    return dict(result.structured_content or {})


def run(question: Question) -> Outcome:
    payload = call(question.tool, question.arguments)
    failures = []
    # `ok` is the report's own verdict -- a failing check_urdf is a correct answer, not
    # a broken tool -- so what marks a soft error is `error`, which only failures carry.
    if payload.get("error"):
        return Outcome(
            question=question, passed=False, failures=[f"tool failed: {payload['error']}"]
        )
    for expectation in question.expect:
        try:
            found = resolve(payload, expectation.path)
        except (KeyError, IndexError, TypeError) as error:
            failures.append(f"{expectation.path}: {error}")
            continue
        if not matches(found, expectation):
            failures.append(f"{expectation.path}: got {found!r}, wanted {expectation.value!r}")
    return Outcome(question=question, passed=not failures, failures=failures)


def matches(found: Any, expectation: Expectation) -> bool:
    if expectation.tol is None:
        return bool(found == expectation.value)
    if not isinstance(found, int | float) or isinstance(found, bool):
        return False
    return abs(float(found) - float(expectation.value)) <= expectation.tol


def resolve(payload: Any, path: str) -> Any:
    """Read one value out of a tool result.

    ``joints[joint=joint_2].cases[1].margin`` -- dotted keys, list indices, and
    ``[field=value]`` to pick the entry a human would name rather than the one
    that happens to be third. ``len path`` gives the length instead of the value,
    which is how "how many would fit?" is asked.
    """
    if path.startswith("len "):
        return len(resolve(payload, path[4:].strip()))
    current = payload
    for key, selector in _steps(path):
        current = current[key] if selector is None else _select(current, selector, path)
    return current


#: A path is keys and selectors, and a selector may contain dots (``payload_kg=0.1``),
#: which is exactly why this is a scanner and not ``path.split(".")``.
_STEP = re.compile(r"\.?(?P<key>[^.\[\]]+)|\[(?P<selector>[^\]]*)\]")


def _steps(path: str) -> list[tuple[str, str | None]]:
    steps, position = [], 0
    for match in _STEP.finditer(path):
        if match.start() != position:
            raise KeyError(f"{path}: cannot read {path[position:]!r}")
        position = match.end()
        steps.append((match.group("key"), match.group("selector")))
    if position != len(path):
        raise KeyError(f"{path}: cannot read {path[position:]!r}")
    return steps


def _select(items: Any, selector: str, path: str) -> Any:
    if not isinstance(items, list):
        raise TypeError(f"{path}: [{selector}] needs a list, got {type(items).__name__}")
    field, is_match, wanted = selector.partition("=")
    if not is_match:
        return items[int(field)]
    for entry in items:
        if entry.get(field) is not None and str(entry[field]) == wanted:
            return entry
    raise KeyError(f"no entry with {field} == {wanted}")
