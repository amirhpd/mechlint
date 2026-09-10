"""One result object, four ways of showing it.

Every command computes a pydantic model and nothing else; this module turns
that model into the terminal table, the JSON an MCP client or CI reads, and the
markdown a pull request can carry. Keeping the rendering here is what makes the
three doors -- terminal, chat, CI -- show the same numbers by construction.
"""

from __future__ import annotations

import json

from pydantic import BaseModel, computed_field, model_validator

from mechlint.core.actuators import Actuator
from mechlint.core.checks import CHECKS, CheckReport, Severity
from mechlint.core.inertia import InertiaReport
from mechlint.core.torque import TorqueCase, TorqueReport

SEVERITY_ORDER = {Severity.FAIL: 0, Severity.WARN: 1, Severity.INFO: 2}


def to_json(report: BaseModel | list[BaseModel]) -> str:
    """Any result object as JSON -- including a plain list, which ``actuators`` returns."""
    if isinstance(report, list):
        return json.dumps([item.model_dump(mode="json") for item in report], indent=2)
    return report.model_dump_json(indent=2)


# --------------------------------------------------------------------------- checks


def render_checks(report: CheckReport, *, fixes: bool = True) -> str:
    header = [
        f"{report.robot} — {report.source}",
        f"model scale: {report.model_scale:g} m per URDF unit",
        "",
    ]
    if not report.findings:
        return "\n".join([*header, "no findings", *_skipped(report.skipped)])

    findings = sorted(report.findings, key=lambda f: (SEVERITY_ORDER[f.severity], f.check))
    width = max(len(f.subject) for f in findings)
    rows = [
        f"  {f.check}  {f.severity.value:<4}  {f.subject:<{width}}  {f.message}" for f in findings
    ]

    tally = report.counts()
    summary = ", ".join(f"{tally[s]} {s.value}" for s in Severity if tally[s])
    body = [*header, *rows, "", f"  {summary or 'nothing found'}"]

    if fixes:
        seen = sorted({f.check for f in findings})
        body += ["", "how to fix:"]
        body += [f"  {check}  {CHECKS[check].fix}" for check in seen]
    return "\n".join([*body, *_skipped(report.skipped)])


def _skipped(skipped: dict[str, str]) -> list[str]:
    if not skipped:
        return []
    return ["", "not run:"] + [f"  {key}  {reason}" for key, reason in sorted(skipped.items())]


def checks_markdown(report: CheckReport) -> str:
    lines = [
        f"# mechlint — {report.robot}",
        "",
        f"`{report.source}` · model scale {report.model_scale:g} m per URDF unit · "
        f"**{'pass' if report.ok else 'fail'}**",
        "",
        "| ID | Severity | Subject | Finding |",
        "|---|---|---|---|",
    ]
    findings = sorted(report.findings, key=lambda f: (SEVERITY_ORDER[f.severity], f.check))
    lines += [
        f"| [{f.check}](checks.md) | {f.severity.value} | `{f.subject}` | {f.message} |"
        for f in findings
    ] or ["| — | — | — | no findings |"]
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------- inertia


def render_inertia(report: InertiaReport) -> str:
    links = [link for link in report.links if link.mass_source != "none"]
    header = [
        f"{report.robot} — {report.source}",
        f"model scale: {report.model_scale:g} m per URDF unit",
        "",
        f"  {'link':<26}{'mass':>10}  {'source':<10}{'centre of mass (m)':<28}{'in URDF':>10}",
    ]
    rows = []
    for link in links:
        com = "(" + ", ".join(f"{v:+.4f}" for v in link.com_m) + ")"
        declared = "—" if link.urdf_mass_kg is None else f"{link.urdf_mass_kg * 1e3:.1f} g"
        flag = " *" if link.convex_hull_fallback else ""
        rows.append(
            f"  {link.link:<26}{link.mass_kg * 1e3:>8.1f} g  "
            f"{link.mass_source:<10}{com:<28}{declared:>10}{flag}"
        )

    # Two totals, because one is misleading. The base and anything bolted to the world
    # beside the robot are in the model but held up by nothing, so adding them into a
    # single figure invites reading a tripod-mounted camera as part of the arm.
    footer = [
        "",
        f"  {'carried by a joint':<26}{report.carried_mass_kg * 1e3:>8.1f} g"
        "   every joint torque comes from this",
        f"  {'not carried':<26}{report.uncarried_mass_kg * 1e3:>8.1f} g"
        f"   {', '.join(report.uncarried_links)}",
        f"  {'total':<26}{report.total_mass_kg * 1e3:>8.1f} g",
    ]
    if any(link.convex_hull_fallback for link in links):
        footer.append("  * convex hull used; the mass is an upper bound (M001)")

    pending = report.pending_components
    if pending:
        labels = ", ".join(sorted({c.label for c in pending}))
        footer += [
            "",
            f"  {len(pending)} component masses missing ({labels}):",
            f"  {pending[0].hint}",
        ]
    return "\n".join([*header, *rows, *footer, *_skipped(report.skipped)])


def inertia_markdown(report: InertiaReport) -> str:
    lines = [
        f"# mechlint inertia — {report.robot}",
        "",
        f"`{report.source}` · model scale {report.model_scale:g} m per URDF unit",
        "",
        "| Link | Mass | Source | Centre of mass (m) | URDF today |",
        "|---|---|---|---|---|",
    ]
    for link in report.links:
        if link.mass_source == "none":
            continue
        com = ", ".join(f"{v:+.4f}" for v in link.com_m)
        declared = "—" if link.urdf_mass_kg is None else f"{link.urdf_mass_kg * 1e3:.1f} g"
        lines.append(
            f"| `{link.link}` | {link.mass_kg * 1e3:.1f} g | {link.mass_source} | "
            f"{com} | {declared} |"
        )
    lines += [
        "",
        f"**Carried by a joint {report.carried_mass_kg * 1e3:.1f} g** — every joint torque comes "
        f"from this. Not carried: {report.uncarried_mass_kg * 1e3:.1f} g "
        f"({', '.join(report.uncarried_links) or 'nothing'}). "
        f"Total {report.total_mass_kg * 1e3:.1f} g.",
    ]
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------- torque


def render_torque(report: TorqueReport) -> str:
    payloads = report.payloads_g
    header = [
        f"{report.robot} — {report.source}",
        f"model scale: {report.model_scale:g} m per URDF unit",
        "  ·  ".join(
            part
            for part in (
                f"gravity {_vector(report.gravity_m_s2)} m/s^2 from the {report.gravity_source}"
                + (f" ({report.implied_mount})" if report.implied_mount else ""),
                f"safety factor {report.safety_factor:g}",
                f"{report.voltage_V:g} V" if report.voltage_V else "",
                f"{report.samples} pose" + ("" if report.samples == 1 else "s"),
                f"payload at {report.payload_at}" if report.payload_at else "",
            )
            if part
        ),
        "",
        f"  {'joint':<9}{'actuator':<14}{'available':>10}"
        + "".join(f"{_payload_label(g):>19}" for g in payloads),
        f"  {'':<9}{'':<14}{'N*m':>10}" + "".join(f"{'N*m  margin':>19}" for _ in payloads),
    ]

    rows = [
        f"  {joint.joint:<9}{joint.actuator_name or '—':<14}"
        + (f"{joint.cases[0].available_Nm:>10.3f}" if joint.judged else f"{'—':>10}")
        + "".join(_cell(case) for case in joint.cases)
        for joint in report.joints
    ]

    footer = ["", "  " + "  ·  ".join(_summary(report))]

    worst = report.worst_joint
    if worst is not None and worst.worst is not None:
        pose = "  ".join(f"{name} {value:+.3f}" for name, value in worst.worst.pose.items())
        footer += [
            "",
            f"  worst pose ({worst.joint}, {_payload_label(worst.worst.payload_kg * 1e3)}): {pose}",
        ]

    footer += [
        "",
        f"  margin = available / (worst torque × safety factor {report.safety_factor:g}); "
        "a joint passes at margin >= 1",
    ]
    # One line per actuator, not per joint: five joints driven by the same servo cite
    # the same datasheet, and printing it five times buries the notes underneath.
    for key in dict.fromkeys(j.actuator for j in report.joints if j.judged):
        joint = next(j for j in report.joints if j.actuator == key)
        driven = [j.joint for j in report.joints if j.actuator == key]
        footer.append(
            f"  {', '.join(driven)}: {joint.actuator_name} {joint.torque_basis}, "
            f"{joint.confidence} confidence — {joint.source_url}"
        )
    for joint in report.joints:
        footer += [f"  {joint.joint}: {note}" for note in joint.notes]
    if report.unjudged:
        footer.append(f"  reported but not judged: {', '.join(report.unjudged)}")

    fixes = sorted({f.check for f in report.findings})
    if fixes:
        footer += ["", "how to fix:"] + [f"  {check}  {CHECKS[check].fix}" for check in fixes]
    return "\n".join([*header, *rows, *footer, *_skipped(report.skipped)])


def _payload_label(grams: float) -> str:
    return "no payload" if grams == 0.0 else f"{grams:g} g"


def _cell(case: TorqueCase) -> str:
    """One payload column: the torque, and the margin with a verdict when judged."""
    if case.ok is None:
        margin = ""
    elif case.margin is None:
        # Gravity applies no torque about this axis, so no actuator can be too weak for it.
        margin = "inf"
    else:
        margin = f"{case.margin:.2f}x" + ("" if case.ok else "!")
    return f"{case.max_torque_Nm:>11.3f}{margin:>8}"


def _summary(report: TorqueReport) -> list[str]:
    parts = [f"{len(report.joints)} joints"]
    judged = [j for j in report.joints if j.judged]
    if judged:
        margins = {
            j.joint: min(c.margin for c in j.cases if c.margin is not None)
            for j in judged
            if any(c.margin is not None for c in j.cases)
        }
        if margins:
            tightest = min(margins, key=lambda name: margins[name])
            parts.append(f"worst margin {margins[tightest]:.2f}x at {tightest}")
        failed = [j.joint for j in judged if j.ok is False]
        parts.append(f"{len(failed)} of {len(judged)} fail" if failed else "all judged joints pass")
    if report.unjudged:
        parts.append(f"{len(report.unjudged)} not judged")
    return parts


def _vector(values: tuple[float, float, float]) -> str:
    return "[" + ", ".join(f"{v:g}" for v in values) + "]"


def torque_markdown(report: TorqueReport) -> str:
    payloads = report.payloads_g
    lines = [
        f"# mechlint torque — {report.robot}",
        "",
        f"`{report.source}` · gravity {_vector(report.gravity_m_s2)} m/s² from the "
        f"{report.gravity_source} · safety factor {report.safety_factor:g}"
        + (f" · {report.voltage_V:g} V" if report.voltage_V else "")
        + f" · {report.samples} pose{'' if report.samples == 1 else 's'}"
        + f" · **{'pass' if report.ok else 'fail'}**",
        "",
        "| Joint | Actuator | Available N·m | "
        + " | ".join(f"{_payload_label(g)} N·m (margin)" for g in payloads)
        + " |",
        "|---|---|---|" + "---|" * len(payloads),
    ]
    for joint in report.joints:
        available = f"{joint.cases[0].available_Nm:.3f}" if joint.judged else "—"
        cells = [
            f"{case.max_torque_Nm:.3f}"
            + (
                ""
                if case.margin is None
                else f" ({case.margin:.2f}× {'pass' if case.ok else '**fail**'})"
            )
            for case in joint.cases
        ]
        lines.append(
            f"| `{joint.joint}` | {joint.actuator_name or '—'} | {available} | "
            + " | ".join(cells)
            + " |"
        )
    lines += ["", "| ID | Severity | Subject | Finding |", "|---|---|---|---|"]
    lines += [
        f"| [{f.check}](checks.md) | {f.severity.value} | `{f.subject}` | {f.message} |"
        for f in report.findings
    ] or ["| — | — | — | nothing exceeds its actuator |"]
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------- actuators


def render_actuators(actuators: list[Actuator], *, voltage: float | None = None) -> str:
    if not actuators:
        return "no actuator in the database meets that requirement"
    keys = max(len(a.key) for a in actuators) + 2
    names = max(len(a.name) for a in actuators) + 2
    lines = [f"  {'key':<{keys}}{'name':<{names}}{'torque':>9}  {'mass':>7}  {'conf':<7}basis"]
    for actuator in actuators:
        limit = actuator.torque_limit(voltage)
        lines.append(
            f"  {actuator.key:<{keys}}{actuator.name:<{names}}{limit.torque_Nm:>7.3f} N*m  "
            f"{actuator.mass_g:>5.0f} g  {actuator.confidence:<7}{limit.basis}"
        )
    lines += [
        "",
        f"  {len(actuators)} actuators"
        + (f" at {voltage:g} V" if voltage is not None else " at their recommended voltage"),
        "  every torque above is a datasheet figure, not a measurement; `conf` is how much "
        "the source is worth",
    ]
    return "\n".join(lines)


def actuator_detail(actuator: Actuator) -> str:
    """Everything known about one actuator, including where it came from."""
    lines = [f"  {label:<16}{value}" for label, value in _detail_rows(actuator)]
    if actuator.note:
        lines += ["", "  " + " ".join(actuator.note.split())]
    return "\n".join(lines)


def _detail_rows(actuator: Actuator) -> list[tuple[str, str]]:
    """One actuator as label/value pairs, so the table and the markdown cannot drift apart.

    The last three rows are the point of the whole database: what the source was, when it
    was read, and what it actually said in its own units.
    """
    rows: list[tuple[str, str]] = [
        ("name", f"{actuator.name} ({actuator.vendor})"),
        ("kind", actuator.kind),
    ]
    if actuator.stall_torque_Nm:
        rows.append(
            (
                "stall torque",
                ", ".join(
                    f"{value:.3f} N*m @ {volts:g} V"
                    for volts, value in sorted(actuator.stall_torque_Nm.items())
                ),
            )
        )
    if actuator.holding_torque_Nm is not None:
        current = f" @ {actuator.rated_current_A:g} A" if actuator.rated_current_A else ""
        rows.append(("holding torque", f"{actuator.holding_torque_Nm:.3f} N*m{current}"))
    if actuator.rated_torque_Nm is not None:
        rows.append(("rated torque", f"{actuator.rated_torque_Nm:.3f} N*m"))
    if actuator.no_load_speed_rpm:
        rows.append(
            (
                "no-load speed",
                ", ".join(
                    f"{value:g} rpm @ {volts:g} V"
                    for volts, value in sorted(actuator.no_load_speed_rpm.items())
                ),
            )
        )
    if actuator.recommended_voltage is not None:
        rows.append(("recommended", f"{actuator.recommended_voltage:g} V"))
    if actuator.voltage_range is not None:
        rows.append(
            ("voltage range", f"{actuator.voltage_range[0]:g} - {actuator.voltage_range[1]:g} V")
        )
    if actuator.gear_ratio is not None:
        rows.append(("gear ratio", f"{actuator.gear_ratio:g}:1"))
    if actuator.axes > 1:
        rows.append(("axes", f"{actuator.axes} (mass below covers the whole unit)"))
    rows.append(("mass", f"{actuator.mass_g:g} g"))
    if actuator.dims_mm is not None:
        rows.append(("dimensions", " × ".join(f"{v:g}" for v in actuator.dims_mm) + " mm"))
    for label, value in (("mounting", actuator.mounting), ("interface", actuator.interface)):
        if value:
            rows.append((label, value))
    if actuator.price_hint:
        rows.append(("price hint", actuator.price_hint))
    rows += [
        ("confidence", actuator.confidence),
        ("source", f"{actuator.source_url} (read {actuator.source_date})"),
        ("as quoted", actuator.source),
    ]
    return rows


def actuators_markdown(actuators: list[Actuator], *, voltage: float | None = None) -> str:
    if not actuators:
        return "No actuator in the database meets that requirement.\n"
    lines = [
        "# mechlint actuators",
        "",
        "Torque at "
        + (f"{voltage:g} V" if voltage is not None else "each entry's recommended voltage")
        + ". Every figure is a datasheet number, never a measurement — `confidence` says how "
        "much the source is worth.",
        "",
        "| Key | Name | Vendor | Torque | Mass | Confidence | Basis | Source |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for actuator in actuators:
        limit = actuator.torque_limit(voltage)
        lines.append(
            f"| `{actuator.key}` | {actuator.name} | {actuator.vendor} | "
            f"{limit.torque_Nm:.3f} N·m | {actuator.mass_g:g} g | {actuator.confidence} | "
            f"{limit.basis} | [datasheet]({actuator.source_url}) |"
        )
    return "\n".join(lines) + "\n"


def actuator_detail_markdown(actuator: Actuator) -> str:
    lines = [
        f"# {actuator.name}",
        "",
        f"`{actuator.key}` · {actuator.vendor} · {actuator.kind} · "
        f"**{actuator.confidence}** confidence",
        "",
    ]
    lines += [f"- **{label}** — {value}" for label, value in _detail_rows(actuator)]
    if actuator.note:
        lines += ["", " ".join(actuator.note.split())]
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------- everything


class FullReport(BaseModel):
    """Both reports under one exit code -- what ``mechlint check`` is.

    A model rather than two printed tables, because CI and an MCP client read
    this door as JSON and need one object with one ``ok`` in it.
    """

    checks: CheckReport
    torque: TorqueReport | None = None

    @model_validator(mode="after")
    def _torque_is_no_longer_skipped(self) -> FullReport:
        """``urdf-check`` lists T001 as "run `mechlint torque`". Here it just ran."""
        if self.torque is not None and "T001" in self.checks.skipped:
            self.checks = self.checks.model_copy(
                update={"skipped": {k: v for k, v in self.checks.skipped.items() if k != "T001"}}
            )
        return self

    @computed_field  # type: ignore[prop-decorator]
    @property
    def ok(self) -> bool:
        return self.checks.ok and (self.torque is None or self.torque.ok)


def render_full(report: FullReport) -> str:
    parts = [render_checks(report.checks)]
    if report.torque is not None:
        parts += ["", "=" * 78, "", render_torque(report.torque)]
    return "\n".join(parts)


def full_markdown(report: FullReport) -> str:
    parts = [checks_markdown(report.checks)]
    if report.torque is not None:
        parts.append(torque_markdown(report.torque))
    return "\n".join(parts)
