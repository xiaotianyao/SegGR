#!/usr/bin/env python3
"""Adaptive (non-uniform) block cut-line computation for blocked-GRADP v2.

Replaces the uniform spatial grid of blocked_partition.task_block with a
kd-tree style partition: G-1 global x cuts, then G-1 y cuts per x column.
Two criteria, per direction, solved as a 1D DP:

  1. (hard) every region's group count within a tolerance band around the
     ideal share (a group = softmax group ~ a net; groups cut by a
     boundary are counted in every region they touch, so the band is
     auto-relaxed if infeasible);
  2. (soft) minimize boundary crossings, which is the 1D proxy for carry
     entries (a group's carry count grows with the number of cells its
     span touches).

The output is a block_fn compatible with blocked_partition.task_block, to
be injected into build_blocked_partition_v2(block_fn=...). Everything
downstream (window membership, carry plan, image build) is unchanged.
"""

from __future__ import annotations

import sys
import time as _time
from bisect import bisect_right
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

_CKPT = Path(__file__).resolve().parents[1] / "cocotb" / "ckpt"
if str(_CKPT) not in sys.path:
    sys.path.insert(0, str(_CKPT))
from fixed_point_model import _sect  # noqa: E402

CHUNK = 64  # task chunk size used by the case builder (CHUNK_SIZE)


def task_rect(task: dict) -> Tuple[int, int, int, int]:
    """Task's coordinate rect in GCell units: (x0, x1, y0, y1), inclusive."""
    line = int(task["line_index"])
    chunk = int(task["chunk_index"])
    if task["task_type"] == "row":
        return chunk * CHUNK, chunk * CHUNK + CHUNK - 1, line, line
    return line, line, chunk * CHUNK, chunk * CHUNK + CHUNK - 1


def task_point(task: dict) -> Tuple[int, int]:
    """Representative point, mirroring task_block's (tx, ty) extraction."""
    x0, _, y0, _ = task_rect(task)
    return x0, y0


def compute_group_spans(
    tasks: Sequence[dict],
    loc_to_candidate: Dict[Tuple[int, int], int],
    p_index_full: Sequence[int],
    candidate_count: int,
) -> List[List[int]]:
    """Per-group [xmin, xmax, ymin, ymax] over the group's touched rects."""
    from blocked_partition import _task_query_candidates

    spans: List[Optional[List[int]]] = [None] * candidate_count
    for task in tasks:
        x0, x1, y0, y1 = task_rect(task)
        for c in _task_query_candidates(task, loc_to_candidate):
            s = spans[c]
            if s is None:
                spans[c] = [x0, x1, y0, y1]
            else:
                s[0] = min(s[0], x0)
                s[1] = max(s[1], x1)
                s[2] = min(s[2], y0)
                s[3] = max(s[3], y1)
    groups: Dict[int, List[int]] = {}
    groups_empty: set = set()
    for c in range(candidate_count):
        g = int(p_index_full[c])
        s = spans[c]
        if g not in groups:
            if s is not None:
                groups[g] = list(s)
            else:
                groups[g] = None
                groups_empty.add(g)
        elif s is not None:
            gs = groups[g]
            if gs is None:
                groups[g] = list(s)
                groups_empty.discard(g)
            else:
                gs[0] = min(gs[0], s[0])
                gs[1] = max(gs[1], s[1])
                gs[2] = min(gs[2], s[2])
                gs[3] = max(gs[3], s[3])
    return [groups[g] for g in sorted(groups.keys()) if g not in groups_empty]


def _dp_cut_1d(
    spans: Sequence[Tuple[int, int]],
    n_parts: int,
    tol: float,
) -> Tuple[List[int], List[int], float]:
    """1D balanced min-crossing partition.

    spans: inclusive [lo, hi] intervals. Returns (cuts, weights, used_tol):
    cuts are n_parts-1 boundary positions (cut before coordinate p), and
    weights are the per-region touched-group counts. The balance band
    [ideal*(1-tol), ideal*(1+tol)] is auto-relaxed (upper, then lower)
    until feasible; used_tol reports the band actually used.
    """
    n = len(spans)
    maxc = max((hi for _, hi in spans), default=0)
    m = maxc + 1  # coordinates 0..maxc; regions are [i, j) over positions

    # lo_le[p] = #{lo <= p}; hi_lt[p] = #{hi < p}; weight(i, j) = #{lo <= j-1} - #{hi < i}
    lo_le = [0] * (m + 1)
    hi_lt = [0] * (m + 1)
    cross = [0] * (m + 1)  # cross[p] = #{lo < p <= hi}, boundary at p
    for lo, hi in spans:
        lo_le[lo] += 1
        hi_lt[hi] += 1
        if lo + 1 <= hi:
            cross[lo + 1] += 1
            if hi + 1 <= maxc:
                cross[hi + 1] -= 1
    for p in range(1, m + 1):
        lo_le[p] += lo_le[p - 1]
        cross[p] += cross[p - 1]
    run = 0
    for p in range(m + 1):
        delta = hi_lt[p]
        hi_lt[p] = run
        run += delta

    def weight(i: int, j: int) -> int:
        return lo_le[j - 1] - hi_lt[i]

    ideal = n / n_parts
    up_factor, down_factor = 1.0, 1.0
    cuts: List[int] = []
    weights: List[int] = []
    for _attempt in range(40):
        hi_w = ideal * (1.0 + tol * up_factor) + 1e-9
        lo_w = ideal * (1.0 - tol * down_factor) - 1e-9
        INF = float("inf")
        # dp[k][j] = min crossing cost covering [0, j) with k regions
        dp = [[INF] * (m + 1) for _ in range(n_parts + 1)]
        par = [[-1] * (m + 1) for _ in range(n_parts + 1)]
        dp[0][0] = 0
        for k in range(1, n_parts + 1):
            for j in range(k, m + 1):
                # i descends: weight(i, j) only grows, break past the band
                i = j - 1
                while i >= k - 1:
                    w = weight(i, j)
                    if w > hi_w:
                        break
                    if w >= lo_w and dp[k - 1][i] < INF:
                        cost = dp[k - 1][i] + (cross[i] if i > 0 else 0)
                        if cost < dp[k][j]:
                            dp[k][j] = cost
                            par[k][j] = i
                    i -= 1
        if dp[n_parts][m] < INF:
            # reconstruct
            cuts = []
            j = m
            for k in range(n_parts, 0, -1):
                i = par[k][j]
                if i > 0:
                    cuts.append(i)
                j = i
            cuts.reverse()
            bounds = [0] + cuts + [m]
            weights = [weight(bounds[t], bounds[t + 1]) for t in range(n_parts)]
            used = tol * max(up_factor, down_factor)
            return cuts, weights, used
        # infeasible: relax the upper band first, then the lower
        up_factor *= 1.25
        if up_factor > 4.0:
            down_factor *= 1.25
    raise RuntimeError(
        f"dp_cut_1d: infeasible for n={n} n_parts={n_parts} even after relaxation"
    )


def compute_adaptive_cuts(
    tasks: Sequence[dict],
    loc_to_candidate: Dict[Tuple[int, int], int],
    p_index_full: Sequence[int],
    candidate_count: int,
    G: int,
    xmax: int,
    ymax: int,
    tol: float = 0.2,
) -> dict:
    """kd-tree cuts: global x cuts, then per-column y cuts."""
    _t = _time.time()
    group_spans = compute_group_spans(tasks, loc_to_candidate, p_index_full, candidate_count)
    _sect("cuts.group_spans", _t)

    _t = _time.time()
    x_spans = [(s[0], s[1]) for s in group_spans]
    x_cuts, x_weights, x_tol = _dp_cut_1d(x_spans, G, tol)
    _sect("cuts.dp_x", _t)

    _t = _time.time()
    y_cuts_per_col: List[List[int]] = []
    y_weights_per_col: List[List[int]] = []
    y_tol_max = tol
    x_bounds = [0] + x_cuts + [max((s[1] for s in x_spans), default=0) + 1]
    for col in range(G):
        x0, x1 = x_bounds[col], x_bounds[col + 1]
        col_spans = [
            (s[2], s[3])
            for s in group_spans
            if s[0] <= x1 - 1 and s[1] >= x0
        ]
        cuts, weights, used_tol = _dp_cut_1d(col_spans, G, tol)
        y_cuts_per_col.append(cuts)
        y_weights_per_col.append(weights)
        y_tol_max = max(y_tol_max, used_tol)
    _sect("cuts.dp_y", _t)

    return {
        "G": G,
        "x_cuts": x_cuts,
        "x_weights": x_weights,
        "y_cuts_per_col": y_cuts_per_col,
        "y_weights_per_col": y_weights_per_col,
        "x_tol_used": x_tol,
        "y_tol_max_used": y_tol_max,
        "group_count": len(group_spans),
    }


def make_block_fn(cuts: dict, xmax: int, ymax: int):
    """Build a task_block-compatible assignment function from cut tables."""
    G = int(cuts["G"])
    x_cuts = list(cuts["x_cuts"])
    y_cuts_per_col = [list(c) for c in cuts["y_cuts_per_col"]]

    def block_fn(task: dict, _G: int, _xmax: int, _ymax: int) -> int:
        x, y = task_point(task)
        bx = min(bisect_right(x_cuts, x), G - 1)
        by = min(bisect_right(y_cuts_per_col[bx], y), G - 1)
        return bx * G + by

    return block_fn
