---
name: liaoning-h-ball-balance-report
description: Create, revise, audit, and compile the 2026 TI Cup Liaoning H-problem “车载平衡滚球运动控制系统” LaTeX design report. Use for report outlines, score-point mapping, verified technical writing, figures and tables, test plans and data, abstracts, report QA, or converting this repository’s K230/STM32/car-control evidence into competition-ready Chinese prose.
---

# 辽宁 H 题电赛报告

Produce a score-aligned report from traceable project evidence. Treat the problem statement, measured data, source code, and report prose as separate layers.

## Load the right references

Always read [problem-spec.md](references/problem-spec.md) before report work.

- Read [report-blueprint.md](references/report-blueprint.md) when outlining, drafting, planning figures, or designing tests.
- Read [project-evidence-map.md](references/project-evidence-map.md) before making implementation claims from this repository.
- Read [latex-and-qa.md](references/latex-and-qa.md) when creating, editing, compiling, or delivering LaTeX/PDF.

## Enforce evidence states

Classify every numerical or performance claim:

- `VERIFIED`: directly supported by current source, schematic, datasheet, or an inspected artifact.
- `MEASURED`: supported by a named raw log, video, CSV, or repeatable test record.
- `PENDING`: required evidence has not been collected.
- `PROPOSED`: design intent or future work, not implemented performance.

Never turn a requirement threshold, configured parameter, simulation result, placeholder, or expected behavior into a measured achievement. Preserve uncertainty explicitly. Prefer “设计目标为” or “待实测验证” over unsupported success claims.

## Workflow

1. Inspect the current problem statement and report-format notice. The H-problem PDF does not itself specify page count, fonts, anonymity, or binding; obtain the current Liaoning submission notice before declaring those rules final.
2. Inspect the current repository state. Do not rely on README values when source or measured data are newer.
3. Build a score-to-evidence matrix for all 20 report points and requirements 1–6.
4. Draft the argument before prose: claim, mechanism, evidence, figure/table, and limitation for each subsection.
5. Keep the car line-following loop and ball-balancing loop distinct, then explain their coupling through chassis acceleration and disturbance rejection.
6. Design tests before inserting results. Record raw trials; compute summaries without deleting failures.
7. Compile with XeLaTeX, render the final PDF to page images, visually inspect every page, and run `scripts/audit_report.py`.

## Writing rules

- Mirror the scoring terms in headings: 小车循迹控制方案、小车循迹控制理论、摆杆系统控制理论、控制电路及程序流程、测试方案与测试结果.
- Allocate space by score and evidence value, not by implementation effort.
- Lead with the implemented design, then compare only credible alternatives that explain a real choice.
- Use equations only when variables are defined and the equation informs controller or parameter selection.
- Give every figure and table a claim-supporting purpose, label, caption, and in-text reference.
- Show complete requirement-level tests, including units, repeated trials, worst case, pass criterion, and analysis.
- Keep code listings out of the main report unless the current format notice explicitly rewards them; use flowcharts and concise pseudocode instead.
- Do not include school, team member, advisor, or identifying metadata until the current rules explicitly permit it.
- Do not claim the K230 control camera automatically satisfies the separate wireless video transmission/recording requirement.

## LaTeX starter

Use `assets/latex-report/` as the starting tree when asked to create the report. It intentionally contains visible `待填/待测` markers so incomplete evidence cannot look final. Replace them only with verified content.

Run:

```powershell
python scripts/audit_report.py --root <report-dir>
python scripts/audit_report.py --root <report-dir> --pdf <report.pdf> --max-pages <confirmed-limit> --strict
```

Treat any unconfirmed page limit as unknown; do not silently assume the historical “摘要 1 页 + 正文 8 页” convention.

## Delivery gate

Before calling a report final, require:

- all 20 report-score points mapped to visible content;
- all requirements 1–6 represented in the test matrix;
- every result claim linked to raw evidence;
- no unresolved placeholders or accidental identity fields;
- no table of contents unless current rules require one;
- successful XeLaTeX compilation;
- page count checked against the current notice;
- every rendered page visually inspected.

