"""One result object, four ways of showing it.

Every command computes a pydantic model and nothing else; this module turns
that model into the terminal table, the JSON an MCP client or CI reads, and the
markdown a pull request can carry. Keeping the rendering here is what makes the
three doors -- terminal, chat, CI -- show the same numbers by construction.
"""

from __future__ import annotations

from mechlint.core.checks import CHECKS, CheckReport, Severity
from mechlint.core.inertia import InertiaReport

SEVERITY_ORDER = {Severity.FAIL: 0, Severity.WARN: 1, Severity.INFO: 2}


def to_json(report: CheckReport | InertiaReport) -> str:
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
