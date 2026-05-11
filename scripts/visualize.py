#!/usr/bin/env python3
"""Visualize a FIRM/FIRMCP run from its results.db.

Reads roadmap, landmarks, start/goal, robot path and time-series tables from
the SQLite database produced by the demo binaries and writes one or more PNG
figures alongside the database.

Usage:
    visualize.py <path/to/results.db> [-o <output_dir>]
    visualize.py <path/to/run_dir>     [-o <output_dir>]
"""

import argparse
import os
import sqlite3
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def fetch(con, table, cols="*"):
    cur = con.cursor()
    try:
        return cur.execute(f"SELECT {cols} FROM {table}").fetchall()
    except sqlite3.OperationalError:
        return []


def plot_roadmap(con, out_path, title):
    nodes = fetch(con, "roadmap_nodes",
                  "id, x, y, theta, cov_xx, cov_yy, is_start, is_goal")
    edges = fetch(con, "roadmap_edges", "source, target, cost, success_prob")
    landmarks = fetch(con, "landmarks", "id, x, y")
    start = fetch(con, "start_state", "x, y")
    goal = fetch(con, "goal_state", "x, y")
    bounds = fetch(con, "bounds", "low_x, high_x, low_y, high_y")
    path = fetch(con, "robot_path", "step, x, y")

    if not nodes and not path:
        print(f"[viz] no roadmap or path in {out_path}; skipping")
        return False

    node_xy = {n[0]: (n[1], n[2]) for n in nodes}

    fig, ax = plt.subplots(figsize=(10, 10))

    if edges and node_xy:
        max_p = max((e[3] for e in edges), default=1.0) or 1.0
        segments, widths = [], []
        for src, tgt, _cost, prob in edges:
            if src in node_xy and tgt in node_xy:
                segments.append((node_xy[src], node_xy[tgt]))
                widths.append(0.3 + 1.2 * (prob / max_p))
        if segments:
            ax.add_collection(LineCollection(
                segments, colors="#bbbbbb", linewidths=widths, alpha=0.4,
                zorder=1, label="roadmap edges"))

    if nodes:
        xs = [n[1] for n in nodes]
        ys = [n[2] for n in nodes]
        cov = [(n[4] + n[5]) for n in nodes]
        sc = ax.scatter(xs, ys, c=cov, s=12, cmap="viridis",
                        zorder=2, label="roadmap nodes")
        cbar = plt.colorbar(sc, ax=ax, fraction=0.04, pad=0.02)
        cbar.set_label("trace(cov_xy)")

    if landmarks:
        ax.scatter([l[1] for l in landmarks], [l[2] for l in landmarks],
                   marker="*", s=180, c="orange", edgecolors="black",
                   zorder=4, label="landmarks")

    if path:
        px = [p[1] for p in path]
        py = [p[2] for p in path]
        ax.plot(px, py, "-", color="crimson", linewidth=2.0,
                zorder=3, label=f"robot path ({len(path)} steps)")

    if start:
        ax.scatter([s[0] for s in start], [s[1] for s in start],
                   marker="o", s=160, c="lime", edgecolors="black",
                   zorder=5, label="start")
    if goal:
        ax.scatter([g[0] for g in goal], [g[1] for g in goal],
                   marker="X", s=180, c="red", edgecolors="black",
                   zorder=5, label="goal")

    if bounds:
        lx, hx, ly, hy = bounds[0]
        ax.set_xlim(lx, hx)
        ax.set_ylim(ly, hy)

    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title)
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"[viz] wrote {out_path}")
    return True


def plot_history(con, out_path, table, columns, ylabel, title):
    rows = fetch(con, table, ", ".join(columns))
    if not rows:
        return False
    fig, ax = plt.subplots(figsize=(10, 4))
    ts = [r[0] for r in rows]
    for i, col in enumerate(columns[1:], start=1):
        ax.plot(ts, [r[i] for r in rows], label=col)
    ax.set_xlabel("timestep")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    if len(columns) > 2:
        ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"[viz] wrote {out_path}")
    return True


def visualize(db_path, out_dir):
    if not os.path.isfile(db_path):
        print(f"[viz] database not found: {db_path}", file=sys.stderr)
        return 1

    os.makedirs(out_dir, exist_ok=True)
    name = os.path.basename(os.path.dirname(db_path)) or "results"
    con = sqlite3.connect(db_path)
    try:
        plot_roadmap(con, os.path.join(out_dir, "roadmap.png"),
                     f"FIRM roadmap — {name}")
        plot_history(con, os.path.join(out_dir, "cost_history.png"),
                     "cost_history", ["timestep", "cost_cov", "cost"],
                     "value", f"Cost history — {name}")
        plot_history(con, os.path.join(out_dir, "success_probability.png"),
                     "success_probability", ["timestep", "probability"],
                     "P(success)", f"Success probability — {name}")
        plot_history(con, os.path.join(out_dir, "velocity.png"),
                     "velocity", ["timestep", "velocity"],
                     "velocity (m/s)", f"Velocity — {name}")
    finally:
        con.close()
    return 0


def resolve_db(target):
    if os.path.isdir(target):
        candidates = [
            os.path.join(target, "results.db"),
            *(os.path.join(target, d, "results.db")
              for d in sorted(os.listdir(target))
              if d.startswith("run-") and
              os.path.isdir(os.path.join(target, d))),
        ]
        for c in candidates:
            if os.path.isfile(c):
                return c
        return None
    return target if os.path.isfile(target) else None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("target", help="path to results.db or run/example directory")
    ap.add_argument("-o", "--output", default=None,
                    help="output directory for PNGs (default: alongside the DB)")
    args = ap.parse_args()

    db = resolve_db(args.target)
    if db is None:
        print(f"[viz] no results.db under {args.target}", file=sys.stderr)
        return 1
    out = args.output or os.path.dirname(db)
    return visualize(db, out)


if __name__ == "__main__":
    sys.exit(main())
