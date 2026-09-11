# The mechanics Q&A benchmark

Sixteen questions about DAST-1, each one tied to the tool call that answers it and the
number that answer has to contain. It exists because "the LLM said 1.6 N·m" is worth
nothing on its own: what makes the answer trustworthy is that the number came from a
tool, and that the tool still returns it tomorrow.

There are two halves, and only the first one runs in CI.

## 1. The validity gate — no LLM

```bash
uv run pytest tests/bench            # as tests, one per question
uv run python -m tests.bench.run     # as a table, with the question text
uv run python -m tests.bench.run mg996r_at_joint_2   # just one
```

The harness calls the MCP tools directly, so there is no API key, no network and no
model. A failure here means the physics moved or a tool's JSON changed shape — both of
which would otherwise surface as a chat that quietly answers differently.

## 2. The manual half — with an LLM

The point of M3 is not that the tools return numbers; it is that a host model *reaches
for them* instead of guessing. That part needs a human to read the transcript, so it is
not automated:

1. Open Claude Code in `dast_1`, which has the `.mcp.json` pointing at `mechlint-mcp`.
2. Ask the questions in `questions.yaml` in your own words, one at a time.
3. For each answer, check three things:
   - it **called a tool** rather than reasoning from the URDF text;
   - the number it quotes **matches** the `expect` block here;
   - it repeats the qualifiers — the safety factor, the datasheet's `confidence`, that
     torque is static, and any override it applied.

A model that gets the right number by arithmetic of its own has failed this, even though
the number is right. `docs/llms.md` is the policy it is being held to.

## Adding a question

Add an entry to `questions.yaml`. The `path` is read out of the tool's JSON result:
dotted keys, `[0]` for an index, `[field=value]` to pick the entry a person would name,
and a leading `len ` for a count.

```yaml
- id: my_question
  question: In the words someone would actually ask.
  tool: torque_budget
  arguments: { config: "{dast1}/mechlint.yaml", payload_g: [100.0] }
  expect:
    - { path: "joints[joint=joint_2].cases[payload_kg=0.1].ok", value: false }
```

Give every torque a `tol`; leave it off for a verdict, a name or a list, where an exact
match is what you mean. `{dast1}` expands to the pinned fixture directory, so a question
never depends on a checked-out copy of the robot.
