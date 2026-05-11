#!/usr/bin/env python3
"""
Sensor Fault Detection using Zero Holonomy Consensus (ZHC).

N sensors measuring the same physical quantity.

DUAL METHOD:
1. ZHC detects THAT a fault exists (non-zero holonomy indicates broken consensus).
2. Per-node edge-weight analysis identifies WHICH sensor is faulty (the one whose
   edges to all others have the lowest weights = most dissimilar readings).

Usage:
    python sensors.py                         # 5 sensors, 1 faulty (+20%)
    python sensors.py --scenario dual         # 10 sensors, 2 faulty
    python sensors.py --scenario triangle     # 3 sensors (can't localize)
"""

import sys
import math
import random
import argparse

from fleet_math import ConstraintGraph


# ── Detection Logic ──────────────────────────────────────────────────────

def build_sensor_graph(readings, scale=2.0):
    """Complete weighted graph from sensor readings.

    weight = exp(-|diff| / scale)
    """
    g = ConstraintGraph()
    for sid, _ in readings:
        g.add_node(sid)
    for i in range(len(readings)):
        for j in range(i + 1, len(readings)):
            diff = abs(readings[i][1] - readings[j][1])
            w = math.exp(-diff / scale)
            g.add_edge(readings[i][0], readings[j][0], weight=max(1e-4, w))
    return g


def mean_similarity(graph, node_id):
    """Mean weight of all edges incident to this node."""
    total, cnt = 0.0, 0
    for u, v in graph.edges:
        if u == node_id or v == node_id:
            total += graph.weight(u, v)
            cnt += 1
    return total / max(cnt, 1)


def detect_faults(readings, tol=0.05, scale=2.0, verbose=True):
    """Detect faulty sensors via ZHC + per-node edge analysis.

    Returns (faulty_ids, scores, graph).
    """
    if isinstance(readings[0], (int, float)):
        readings = [(f"s{i}", v) for i, v in enumerate(readings)]

    ids, vals = zip(*readings)
    n = len(readings)
    mean = sum(vals) / n
    std = math.sqrt(sum((v - mean)**2 for v in vals) / n) if n > 1 else 0

    # Build sensor graph
    graph = build_sensor_graph(readings, scale)
    consensus, violations = graph.check_consensus(tolerance=tol)

    if verbose:
        print(f"\n{'='*60}")
        print(f"ZHC Sensor Fault Detection")
        print(f"Sensors: {n}, Scale: {scale} (weight=0.5 at diff≈{scale*0.69:.2f})")
        print(f"{'='*60}\n")
        print("Readings:")
        for sid, val in readings:
            z = abs(val - mean) / max(std, 1e-10)
            flag = " ⚠️" if z > 2.5 else ""
            print(f"  {sid}: {val:8.4f}  (z={z:.2f}){flag}")
        print(f"  mean={mean:.4f}  std={std:.4f}\n")

    # ── Per-node edge similarity analysis ──
    # A faulty sensor has low similarity to ALL honest ones, pulling
    # its mean incident weight down sharply.
    sim_scores = {node: mean_similarity(graph, node) for node in ids}

    # Normalize: lower similarity = higher fault probability
    max_sim = max(sim_scores.values()) if sim_scores else 1
    fault_scores = {}
    for node in ids:
        s = sim_scores[node]
        # Score = 1 - (similarity / max_similarity)
        fault_scores[node] = round(1.0 - s / max_sim, 3)

    sorted_fault = sorted(fault_scores.items(), key=lambda x: -x[1])

    # Classify: clear gap in similarity scores
    sorted_sim = sorted(sim_scores.items(), key=lambda x: x[1])
    faulty = []
    if len(sorted_sim) >= 3:
        lowest, second = sorted_sim[0][1], sorted_sim[1][1]
        if lowest < 0.3 or second > lowest * 1.5:
            faulty.append(sorted_sim[0][0])
        if len(sorted_sim) >= 4:
            # Check for second fault: needs even clearer gap
            third = sorted_sim[2][1]
            if second < 0.3 or sorted_sim[1][1] < sorted_sim[2][1] * 0.5:
                faulty.append(sorted_sim[1][0])

    if verbose:
        h_s = [f"{h:.4f}" for _, h in violations[:5]]
        print(f"ZHC: {'✅ consensus' if consensus else '⚠️  BROKEN'} "
              f"({len(violations)} violations)")
        print(f"     Top holonomy: [{', '.join(h_s)}]\n")
        print(f"Per-node similarity analysis:")
        for node, sim in sorted_sim:
            flag = " ⚠️ FAULTY" if node in faulty else ""
            print(f"  {node}: mean_similarity={sim:.4f} "
                  f"(fault_score={fault_scores[node]:.1%}){flag}")

        if faulty:
            h = sum(vals[i] for i in range(n) if ids[i] not in faulty) / max(n - len(faulty), 1)
            print(f"\n  → ZHC detected broken consensus.")
            print(f"  → Node analysis flags: {faulty}")
            for sid in faulty:
                i = ids.index(sid)
                print(f"     {sid} reading={vals[i]:.4f} vs honest mean={h:.4f} "
                      f"(diff={abs(vals[i]-h):.3f})")

    return faulty, fault_scores, graph


# ── Scenarios ────────────────────────────────────────────────────────────

def scenario_basic(n=5, bias_pct=20.0, seed=42):
    random.seed(seed)
    true = 25.0
    bi = n // 2
    bias = true * bias_pct / 100.0
    r = [(f"s{i}", true + (bias if i == bi else 0) + random.gauss(0, 0.3)) for i in range(n)]
    return r, [bi]


def scenario_dual(num=10, seed=42):
    random.seed(seed)
    true = 25.0
    fi = [num // 3, 2 * num // 3]
    biases = [0.20, -0.15]
    r = []
    for i in range(num):
        b = true * (biases[0] if i == fi[0] else biases[1] if i == fi[1] else 0)
        r.append((f"s{i}", true + b + random.gauss(0, 0.3)))
    return r, fi


def scenario_triangle(seed=42):
    random.seed(seed)
    true = 100.0
    return [("s0", true + random.gauss(0, 0.5)),
            ("s1", true + random.gauss(0, 0.5)),
            ("s2", true + 20.0 + random.gauss(0, 0.5))], [2]


# ── Main ─────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--num-sensors", type=int, default=5)
    p.add_argument("--bias", type=float, default=20.0)
    p.add_argument("--tol", type=float, default=0.05)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--scenario", choices=["basic", "dual", "triangle"], default="basic")
    args = p.parse_args()

    if args.scenario == "triangle":
        print(f"\n{'#'*60}")
        print("# 3-Sensor Triangle Limitation")
        print(f"{'#'*60}")
        print("A 3-sensor triangle has only 1 fundamental cycle. ZHC can")
        print("detect consensus is broken, but cannot localize the fault")
        print("within a single 3-node cycle. Per-node similarity can still")
        print("suggest a candidate.\n")
        readings, actual = scenario_triangle(args.seed)
    elif args.scenario == "dual":
        readings, actual = scenario_dual(args.num_sensors, args.seed)
    else:
        readings, actual = scenario_basic(args.num_sensors, args.bias, args.seed)

    faulty, scores, graph = detect_faults(readings, tol=args.tol)

    actual_ids = [readings[i][0] for i in actual]
    print(f"\n{'─'*60}")
    print(f"Ground truth: {actual_ids}")
    print(f"ZHC+Node analysis: {faulty}")
    print(f"{'─'*60}")
    if set(faulty) == set(actual_ids):
        print("✅ Perfect detection")
    elif faulty and set(faulty).issuperset(set(actual_ids)):
        print("⚠️  Detected all faults + false positives")
    elif faulty and set(faulty).issubset(set(actual_ids)):
        print("⚠️  Partial — missed some")
    elif not faulty and actual_ids:
        print("⚠️  ZHC detected fault but localization is ambiguous")
    else:
        print("❌ Detection failed")


if __name__ == "__main__":
    main()
