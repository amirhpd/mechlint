"""The benchmark as a test: every question in ``questions.yaml`` must still answer.

This is the validity gate. It runs in CI with no API key, because there is no
model in it -- only the tools, and the numbers they have to keep producing.
"""

from __future__ import annotations

import pytest

pytest.importorskip("mechlint.mcp.server", reason="needs the optional [mcp] extra")

from tests.bench import harness  # noqa: E402

QUESTIONS = harness.load()


def test_the_benchmark_is_not_empty() -> None:
    """A file that quietly stopped loading would make every test below vacuous."""
    assert len(QUESTIONS) >= 10


@pytest.mark.parametrize("question", QUESTIONS, ids=[q.id for q in QUESTIONS])
def test_the_tools_still_answer(question: harness.Question) -> None:
    outcome = harness.run(question)

    assert outcome.passed, f"{question.question}\n  " + "\n  ".join(outcome.failures)


def test_every_question_names_a_tool_that_exists() -> None:
    """A typo in `tool:` would otherwise fail as 'unknown tool', which reads like a bug."""
    import asyncio

    from mechlint.mcp import server

    names = {tool.name for tool in asyncio.run(server.server.list_tools())}
    assert {question.tool for question in QUESTIONS} <= names


def test_the_headline_question_of_the_milestone_is_in_here() -> None:
    """M3 is done when Claude Code answers this one from the tool, not from memory."""
    assert "mg996r_at_joint_2" in {question.id for question in QUESTIONS}


def test_a_wrong_expectation_fails_rather_than_passing_quietly() -> None:
    """The gate has to be able to fail, or it is decoration."""
    question = next(q for q in QUESTIONS if q.id == "reach")
    broken = question.model_copy(
        update={"expect": [harness.Expectation(path="reach_m", value=99.0, tol=0.001)]}
    )

    assert harness.run(broken).passed is False


def test_an_unreadable_path_is_reported_not_raised() -> None:
    question = next(q for q in QUESTIONS if q.id == "reach")
    broken = question.model_copy(
        update={"expect": [harness.Expectation(path="reach_metres", value=0.759)]}
    )
    outcome = harness.run(broken)

    assert outcome.passed is False
    assert "reach_metres" in outcome.summary
