#!/usr/bin/env python3
"""Build a self-contained benchmarks.zip for third-party review/submission.

External review finding: benchmarks/ ships di_planner.py, di_totg.py,
benchmark_fr3.py, fr3_kinematics.py, fr3_hessian_norm.py, improve_test.py,
benchmark_dynamic.py, mujoco_compare.py, fr3_dynamic_obstacle.py as
importlib-based shims that re-export the real modules from the sibling
avoidance_obstacle/sim/ repo (deliberately -- sim/ is the single source of
truth, shared with the obstacle-avoidance paper, so day-to-day editing
should never fork the code). That's the right structure for local dev, but
it means a plain `zip -r benchmarks.zip benchmarks/` silently omits the
code the shims point to, so it is not reproducible for anyone without both
repos cloned as siblings.

This script does NOT change that dev-time structure. It builds a separate,
frozen SUBMISSION SNAPSHOT: a staging copy of benchmarks/ with each shim
replaced by the real module's actual current content (inlined, not
re-exported), plus a PROVENANCE.txt recording the exact commit each repo
was at. A supplementary-material zip is expected to be a frozen snapshot
anyway, so inlining here trades nothing away -- the live shim/SSOT
architecture in git is untouched.

Usage: python3 make_release_zip.py [output_path]
  (default output: ../benchmarks_standalone.zip, i.e. repo root)
"""
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
AD_REPO = HERE.parent
AO_SIM = AD_REPO.parent / "avoidance_obstacle" / "sim"

# The 9 shims currently in benchmarks/, each pointing at AO_SIM/<name>.
SHIMMED_MODULES = [
    "di_planner.py",
    "di_totg.py",
    "benchmark_fr3.py",
    "fr3_kinematics.py",
    "fr3_hessian_norm.py",
    "improve_test.py",
    "benchmark_dynamic.py",
    "mujoco_compare.py",
    "fr3_dynamic_obstacle.py",
]

EXCLUDE_DIRS = {"__pycache__", ".cache", ".mplconfig"}
EXCLUDE_SUFFIXES = {".pyc"}


def _git_rev(repo: Path) -> str:
    out = subprocess.run(
        ["git", "-C", str(repo), "rev-parse", "HEAD"],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    dirty = subprocess.run(
        ["git", "-C", str(repo), "status", "--short"],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    return out + (" (dirty working tree!)" if dirty else "")


def build(output_path: Path):
    if not AO_SIM.is_dir():
        sys.exit(f"error: {AO_SIM} not found -- clone avoidance_obstacle as a "
                  f"sibling of {AD_REPO.name} to build the release zip.")

    staging = HERE / "_release_staging"
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir()

    for item in HERE.iterdir():
        if item.name in EXCLUDE_DIRS or item.name == staging.name:
            continue
        if item.suffix in EXCLUDE_SUFFIXES:
            continue
        if item.name in SHIMMED_MODULES:
            continue  # replaced below with the real module content
        if item.is_dir():
            shutil.copytree(item, staging / item.name,
                             ignore=shutil.ignore_patterns(*EXCLUDE_DIRS, "*.pyc"))
        else:
            shutil.copy2(item, staging / item.name)

    for name in SHIMMED_MODULES:
        real = AO_SIM / name
        if not real.is_file():
            sys.exit(f"error: shim target {real} not found")
        shutil.copy2(real, staging / name)

    provenance = staging / "PROVENANCE.txt"
    provenance.write_text(
        "Frozen submission snapshot -- built by make_release_zip.py.\n\n"
        f"autonomous_driving @ {_git_rev(AD_REPO)}\n"
        f"avoidance_obstacle @ {_git_rev(AO_SIM.parent)}\n\n"
        "The following files are inlined here from avoidance_obstacle/sim/ "
        "(normally imported there via a live shim during development; "
        "sim/ is the single source of truth shared with the obstacle-"
        "avoidance paper):\n" + "\n".join(f"  - {n}" for n in SHIMMED_MODULES) + "\n"
    )

    if output_path.exists():
        output_path.unlink()
    with zipfile.ZipFile(output_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for f in sorted(staging.rglob("*")):
            if f.is_file():
                zf.write(f, arcname=Path("benchmarks") / f.relative_to(staging))

    shutil.rmtree(staging)
    print(f"wrote {output_path} ({output_path.stat().st_size / 1024:.0f} KiB)")


if __name__ == "__main__":
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else AD_REPO / "benchmarks_standalone.zip"
    build(out)
