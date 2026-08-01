# LaTeX and PDF workflow

## Engine and provisional style

Use XeLaTeX. The starter uses `ctexart`, A4, Chinese Song-style body font, Times New Roman for Latin text when available, a top margin above 3 cm, fixed-style body spacing, and right-aligned page numbers.

These are provisional historical conventions, not confirmed 2026 Liaoning rules. Replace them when the current submission notice is available.

Do not include a table of contents unless required. It consumes space without earning a listed score point.

## Working tree

Copy `assets/latex-report/` into a project report directory. Keep:

```text
report/
  main.tex
  sections/
  figures/
  data/
  build/
```

Keep raw data outside LaTeX tables. Generate plots and computed summaries from raw data whenever practical.

## Compilation

Run XeLaTeX at least twice for references:

```powershell
xelatex -interaction=nonstopmode -halt-on-error -output-directory build main.tex
xelatex -interaction=nonstopmode -halt-on-error -output-directory build main.tex
```

If bibliography tooling is later added, run it between LaTeX passes.

## Visual verification

Render the final PDF:

```powershell
pdftoppm -png -r 150 build/main.pdf build/page
```

Inspect every page for:

- clipped or overlapping text;
- unreadable labels and plots;
- isolated headings;
- excessive white space;
- broken cross-references;
- tables wider than the text block;
- inconsistent units, symbols, captions, and decimal precision;
- accidental identity or metadata;
- page number and top-blank-area compliance.

## Automated audit

Run:

```powershell
python scripts/audit_report.py --root report
python scripts/audit_report.py --root report --pdf report/build/main.pdf --max-pages <confirmed-limit> --strict
```

Add `--forbid "学校名称"` and similar arguments for actual identity strings known to the team.

The script is a guardrail, not a substitute for reading the current rules or visually inspecting the PDF.
