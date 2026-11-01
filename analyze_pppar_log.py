#!/usr/bin/env python3
"""Summarizes PPP-AR diagnostics, fix states, resets, and epoch statistics."""

import argparse
import collections
import pathlib
import re
import sys


FIX_STATE_RE = re.compile(
    r"AR SYS=(?P<sys>\S+) FIX_STATE class=(?P<class>\S+) type=(?P<type>\S+) "
    r".*?WL=(?P<wl>\d+) NL=(?P<nl>\d+) .*?ratio=(?P<ratio>[-+0-9.]+) .*?"
    r"histWlQualified=(?P<hwl>\d+) histNlQualified=(?P<hnl>\d+) "
    r"warmupDropped=(?P<warm>\d+)"
)
ARC_RESET_RE = re.compile(
    r"AR HIST SAT=(?P<sat>\S+) event=arc_reset reason=(?P<reason>\S+) epoch=(?P<epoch>\d+)"
)
HIST_RE = re.compile(
    r"AR HIST SAT=(?P<sat>\S+) lock=(?P<lock>\d+) wlFix=(?P<wl>\d+) nlFix=(?P<nl>\d+) arc=(?P<arc>\d+) fixedNow=(?P<fixed>\S+)"
)
FLOAT_KEEP_RE = re.compile(
    r"AR EPOCH note=float_epoch_history_preserved trackedSats=(?P<count>\d+)"
)
EPOCH_END_RE = re.compile(
    r"AR EPOCH end status=(?P<status>\S+) constraints=(?P<constraints>\d+)"
)


def summarize_log(path: pathlib.Path) -> int:
    fix_counts = collections.Counter()
    type_counts = collections.Counter()
    sys_fix_counts = collections.Counter()
    ratios_by_sys = collections.defaultdict(list)
    hist_wl_by_sys = collections.Counter()
    hist_nl_by_sys = collections.Counter()
    warmup_drop_by_sys = collections.Counter()
    arc_reset_reason_counts = collections.Counter()
    arc_reset_sat_counts = collections.Counter()
    float_keep_epochs = 0
    epoch_status_counts = collections.Counter()
    max_lock_by_sat = {}
    max_wlfix_by_sat = {}
    max_nlfix_by_sat = {}

    with path.open("r", encoding="utf-8", errors="replace") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue

            m = FIX_STATE_RE.search(line)
            if m:
                sys_id = m.group("sys")
                cls = m.group("class")
                fix_type = m.group("type")
                ratio = float(m.group("ratio"))
                hist_wl = int(m.group("hwl"))
                hist_nl = int(m.group("hnl"))
                warmup = int(m.group("warm"))

                fix_counts[cls] += 1
                type_counts[fix_type] += 1
                sys_fix_counts[sys_id] += 1
                ratios_by_sys[sys_id].append(ratio)
                hist_wl_by_sys[sys_id] += hist_wl
                hist_nl_by_sys[sys_id] += hist_nl
                warmup_drop_by_sys[sys_id] += warmup
                continue

            m = ARC_RESET_RE.search(line)
            if m:
                sat = m.group("sat")
                reason = m.group("reason")
                arc_reset_reason_counts[reason] += 1
                arc_reset_sat_counts[sat] += 1
                continue

            m = HIST_RE.search(line)
            if m:
                sat = m.group("sat")
                max_lock_by_sat[sat] = max(max_lock_by_sat.get(sat, 0), int(m.group("lock")))
                max_wlfix_by_sat[sat] = max(max_wlfix_by_sat.get(sat, 0), int(m.group("wl")))
                max_nlfix_by_sat[sat] = max(max_nlfix_by_sat.get(sat, 0), int(m.group("nl")))
                continue

            m = FLOAT_KEEP_RE.search(line)
            if m:
                float_keep_epochs += 1
                continue

            m = EPOCH_END_RE.search(line)
            if m:
                epoch_status_counts[m.group("status")] += 1
                continue

    print(f"log: {path}")
    print()
    print("Epoch Status")
    for status, count in sorted(epoch_status_counts.items()):
        print(f"  {status}: {count}")
    print()

    print("Fix Class Counts")
    for cls, count in sorted(fix_counts.items()):
        print(f"  {cls}: {count}")
    print()

    print("Fix Type Counts")
    for fix_type, count in sorted(type_counts.items()):
        print(f"  {fix_type}: {count}")
    print()

    print("Per-System Summary")
    for sys_id in sorted(sys_fix_counts):
        ratios = ratios_by_sys[sys_id]
        avg_ratio = sum(ratios) / len(ratios) if ratios else 0.0
        print(
            f"  {sys_id}: fixes={sys_fix_counts[sys_id]} avgRatio={avg_ratio:.3f} "
            f"histWlQualified={hist_wl_by_sys[sys_id]} "
            f"histNlQualified={hist_nl_by_sys[sys_id]} "
            f"warmupDropped={warmup_drop_by_sys[sys_id]}"
        )
    print()

    print("Arc Reset Reasons")
    for reason, count in sorted(arc_reset_reason_counts.items()):
        print(f"  {reason}: {count}")
    print()

    print(f"Float Epochs With History Preserved: {float_keep_epochs}")
    print()

    def top_items(title, mapping, limit=10):
        print(title)
        ranked = sorted(mapping.items(), key=lambda kv: (-kv[1], kv[0]))[:limit]
        if not ranked:
            print("  -")
        else:
            for key, value in ranked:
                print(f"  {key}: {value}")
        print()

    top_items("Top Arc Reset Satellites", arc_reset_sat_counts)
    top_items("Top Max Lock Epochs", max_lock_by_sat)
    top_items("Top Max WL Fixed Epochs", max_wlfix_by_sat)
    top_items("Top Max NL Fixed Epochs", max_nlfix_by_sat)
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Summarize RT-PPP PPP-AR log behavior.")
    parser.add_argument("logfile", type=pathlib.Path, help="Path to PPP-AR log file")
    args = parser.parse_args(argv)

    if not args.logfile.exists():
        print(f"error: file not found: {args.logfile}", file=sys.stderr)
        return 2
    return summarize_log(args.logfile)


if __name__ == "__main__":
    raise SystemExit(main())
