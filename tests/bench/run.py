"""``python -m tests.bench.run`` -- the benchmark as a script, for a human to read.

Same questions and the same harness as ``pytest tests/bench``; this prints them
as a table with the question text, which is what makes a failure legible ("it no
longer knows what to do about joint_2" rather than "assert 1.65 == 1.71"). Exits
non-zero if anything fails, so it also works as a CI step on its own.
"""

from __future__ import annotations

import sys

from tests.bench import harness


def main(argv: list[str] | None = None) -> int:
    only = set(argv or [])
    questions = [q for q in harness.load() if not only or q.id in only]
    if not questions:
        print(f"no question matches {', '.join(sorted(only))}", file=sys.stderr)
        return 2

    outcomes = [harness.run(question) for question in questions]
    width = max(len(outcome.question.id) for outcome in outcomes)
    for outcome in outcomes:
        mark = "PASS" if outcome.passed else "FAIL"
        print(f"{mark}  {outcome.question.id:<{width}}  {outcome.question.question}")
        for failure in outcome.failures:
            print(f"        {failure}")

    failed = [outcome for outcome in outcomes if not outcome.passed]
    print(f"\n{len(outcomes) - len(failed)}/{len(outcomes)} answered")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
