#!/usr/bin/env python3
"""Audit a Liaoning H-problem LaTeX report for common finalization blockers."""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path


PLACEHOLDER_PATTERNS = {
    "待填/待测 marker": re.compile(r"待填|待测|待补|待确认"),
    "English placeholder": re.compile(r"\b(?:TODO|TBD|PENDING|PLACEHOLDER)\b", re.I),
    "unresolved reference": re.compile(r"\?\?"),
}

REQUIRED_TERMS = (
    "摘要",
    "小车循迹控制方案",
    "小车循迹控制理论",
    "摆杆系统控制理论",
    "控制电路",
    "程序流程",
    "测试方案",
    "测试结果",
)


def read_tex_tree(root: Path) -> tuple[str, list[Path]]:
    files = sorted(root.rglob("*.tex"))
    if not files:
        raise FileNotFoundError(f"no .tex files found under {root}")
    chunks = []
    for path in files:
        chunks.append(f"\n% FILE: {path.relative_to(root)}\n")
        chunks.append(path.read_text(encoding="utf-8", errors="replace"))
    return "".join(chunks), files


def pdf_page_count(path: Path) -> int:
    try:
        from pypdf import PdfReader
    except ImportError:
        PdfReader = None
    if PdfReader is not None:
        return len(PdfReader(str(path)).pages)

    executable_names = ("pdfinfo.exe", "pdfinfo") if os.name == "nt" else ("pdfinfo",)
    for directory in os.environ.get("PATH", "").split(os.pathsep):
        if not directory:
            continue
        for name in executable_names:
            candidate = Path(directory) / name
            if not candidate.is_file():
                continue
            result = subprocess.run(
                [str(candidate), str(path)],
                check=False,
                capture_output=True,
                text=True,
                errors="replace",
            )
            match = re.search(r"^Pages:\s*(\d+)\s*$", result.stdout, re.MULTILINE)
            if result.returncode == 0 and match:
                return int(match.group(1))

    raise RuntimeError("PDF page check requires pypdf or a working pdfinfo executable")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, type=Path, help="report source root")
    parser.add_argument("--pdf", type=Path, help="compiled PDF")
    parser.add_argument("--max-pages", type=int, help="confirmed total page limit")
    parser.add_argument("--forbid", action="append", default=[], help="forbidden identity text")
    parser.add_argument("--strict", action="store_true", help="fail on warnings")
    args = parser.parse_args()

    root = args.root.resolve()
    text, tex_files = read_tex_tree(root)
    warnings: list[str] = []
    errors: list[str] = []

    if "\\tableofcontents" in text:
        errors.append("table of contents found; remove unless current rules require it")

    for label, pattern in PLACEHOLDER_PATTERNS.items():
        count = len(pattern.findall(text))
        if count:
            warnings.append(f"{label}: {count}")

    for term in REQUIRED_TERMS:
        if term not in text:
            warnings.append(f"required rubric term not found: {term}")

    risky_commands = {
        "\\author{": "author field",
        "\\thanks{": "thanks/identity field",
        "指导教师": "advisor identity label",
        "学号": "student-number label",
    }
    for needle, label in risky_commands.items():
        if needle in text:
            warnings.append(f"possible identity leak: {label}")

    for forbidden in args.forbid:
        if forbidden and forbidden in text:
            errors.append(f"forbidden identity text found: {forbidden!r}")

    if args.max_pages is not None and args.pdf is None:
        errors.append("--max-pages requires --pdf")
    if args.pdf is not None:
        pdf = args.pdf.resolve()
        if not pdf.is_file():
            errors.append(f"PDF not found: {pdf}")
        else:
            pages = pdf_page_count(pdf)
            print(f"PDF pages: {pages}")
            if args.max_pages is not None and pages > args.max_pages:
                errors.append(f"page limit exceeded: {pages} > {args.max_pages}")

    print(f"TeX files: {len(tex_files)}")
    for item in warnings:
        print(f"WARNING: {item}")
    for item in errors:
        print(f"ERROR: {item}")

    if errors or (args.strict and warnings):
        return 1
    print("Audit completed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
