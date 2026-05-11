#!/usr/bin/env python3
"""
Swarm Formation Control using Zero Holonomy Consensus (ZHC).

Simulates N robots forming a geometric pattern. ZHC detects when
formation consensus is broken (non-zero holonomy). Per-node edge-weight
analysis identifies which robot drifted.

CORE INSIGHT: ZHC checks if the product of edge weights around every
cycle equals 1. When a robot drifts, edges connecting it to others
deviate from their ideal weight, causing non-zero holonomy on cycles
containing that robot.

Usage:
    python swarm.py                        # K5 pentagram (rich cycles)
    python swarm.py --shape grid           # 3×4 grid with diagonals
    python swarm.py --shape triangle       # Laman-minimal triangle
"""

import sys
import math
import random
import argparse

from fleet_math import ConstraintGraph as CG


# ── Shape Generators ─────────────────────────────────────────────────────

def gen_pentagram():
    """5 robots in a pentagon with all chords (K5)."""
    N = 5
    angles = [2 * math.pi * i / N - math.pi / 2 for i in range(N)]
    positions = {f"bot_{i}": (math.cos(a), math.sin(a)) for i, a in enumerate(angles)}
    edges = [(f"bot_{i}", f"bot_{j}") for i in range(N) for j in range(i + 1, N)]
    origins = [(math.cos(a), math.sin(a)) for a in angles]
    return N, positions, edges, origins


def gen_grid():
    """10 robots in a 3×4 grid with diagonals."""
    names = [f"bot_{i}" for i in range(10)]
    positions = {f"bot_{i}": ((i % 4) * 2.0, (i // 4) * 2.0) for i in range(10)}
    edges = set()
    for i in range(10):
        r, c = i // 4, i % 4
        if c < 3 and i + 1 < 10:
            edges.add((names[i], names[i + 1]))
        if r < 2 and i + 4 < 10:
            edges.add((names[i], names[i + 4]))
        if c < 3 and r < 2 and i + 5 < 10:
            edges.add((names[i], names[i + 5]))
        if c > 0 and r < 2 and i + 3 < 10:
            edges.add((names[i], names[i + 3]))
    origins = {n: list(positions[n]) for n in names}
    return 10, positions, list(edges), list(origins.values())


def gen_triangle():
    """3 robots in equilateral triangle (1 fundamental cycle)."""
    positions = {
        "bot_0": (0.0, 0.0),
        "bot_1": (2.0, 0.0),
        "bot_2": (1.0, math.sqrt(3)),
    }
    edges = [("bot_0", "bot_1"), ("bot_1", "bot_2"), ("bot_2", "bot_0")]
    origins = [(0.0, 0.0), (2.0, 0.0), (1.0, math.sqrt(3))]
    return 3, positions, edges, origins


SHAPES = {"pentagram": gen_pentagram, "grid": gen_grid, "triangle": gen_triangle}


# ── Detection & Correction ───────────────────────────────────────────────

def formation_agreement(graph, violator_id):
    """Compute mean weight of edges incident to a given node."""
    weights = []
    for u, v in graph.edges:
        if u == violator_id or v == violator_id:
            w = graph.weight(u, v) if u <= v else graph.weight(v, u)
            w = graph.weight(u, v)
            weights.append(w)
    return sum(weights) / max(len(weights), 1)


def find_violator_by_edges(graph, nodes):
    """Find the node with the lowest incident-edge agreement.

    A drifting robot has edges deviating from the formation, so its
    incident edge weights are lowest. This is more reliable than
    spanning-tree-based cycle analysis for localization.
    """
    best_score = float('inf')
    violator = None
    for node in nodes:
        total_w = 0.0
        count = 0
        for u, v in graph.edges:
            if u == node or v == node:
                w = graph.weight(u, v)
                total_w += w
                count += 1
        mean_w = total_w / max(count, 1)
        # Lower mean weight = worse formation agreement
        if mean_w < best_score:
            best_score = mean_w
            violator = node
    return violator, best_score


# ── Simulation ───────────────────────────────────────────────────────────

def run_simulation(
    shape="pentagram", steps=15, drift=0.3, gain=0.6, tol=0.1, vi=None
):
    generator = SHAPES[shape]
    N, positions, edges, origins = generator()

    if vi is None:
        vi = N // 2
    violator_actual = f"bot_{vi}"

    print(f"\n{'='*60}")
    print(f"Swarm Formation Demo: {shape.title()}")
    print(f"Robots: {N}, Edges: {len(edges)}, Drift on: {violator_actual}")
    print(f"{'='*60}\n")

    current = {n: list(positions[n]) for n in positions}
    history = []

    for step in range(steps):
        # ── Drift ──
        for name in current:
            if name == violator_actual:
                current[name][0] += random.gauss(0, drift * 0.8)
                current[name][1] += random.gauss(0, drift * 0.8)
            else:
                current[name][0] += random.gauss(0, drift * 0.02)
                current[name][1] += random.gauss(0, drift * 0.02)

        # ── Build constraint graph ──
        graph = CG()
        for u, v in edges:
            ux, uy = current[u]
            vx, vy = current[v]
            d = math.hypot(vx - ux, vy - uy)
            oi = int(u.split("_")[1])
            oj = int(v.split("_")[1])
            ox = origins[oj][0] - origins[oi][0]
            oy = origins[oj][1] - origins[oi][1]
            dd = math.hypot(ox, oy)
            r = d / dd if dd > 0 else 1.0
            w = math.exp(-2.0 * abs(r - 1.0))
            graph.add_edge(u, v, weight=max(0.05, min(5.0, w)))

        # ── ZHC detection ──
        consensus, violations = graph.check_consensus(tolerance=tol)
        agg_h = sum(abs(h - 1.0) for _, h in violations) / max(len(violations), 1)

        # ── Localization via incident-edge analysis ──
        detected, score = find_violator_by_edges(graph, graph.nodes)

        # ── Correction ──
        if detected and not consensus:
            di = int(detected.split("_")[1])
            ox, oy = origins[di]
            dx = ox - current[detected][0]
            dy = oy - current[detected][1]
            pull = min(gain * agg_h * 0.3, 0.4)
            current[detected][0] += dx * pull
            current[detected][1] += dy * pull

        # ── Log ──
        matches = "✓" if detected == violator_actual else "✗"
        status = "✅ OK" if consensus else "⚠️  DRIFT"
        history.append({
            "step": step, "consensus": consensus,
            "holonomy": round(agg_h, 4),
            "detected": detected, "actual": violator_actual,
            "match": matches, "status": status,
            "violations": len(violations),
        })
        print(
            f"Step {step:2d}: h={agg_h:.4f}  "
            f"detected={detected:>6} {matches}  {status}  "
            f"(violations={len(violations)})"
        )

    # ── Summary ──
    print(f"\n{'─'*60}")
    cs = next((h["step"] for h in history if h["consensus"]), None)
    if cs is not None:
        print(f"✅ Consensus restored at step {cs}")
    else:
        print(f"❌ Not converged (final h={history[-1]['holonomy']})")

    correct = sum(1 for h in history if h["detected"] == violator_actual)
    print(f"Localization accuracy: {correct}/{steps} steps ({correct*100//steps}%)")
    print(f"Mean holonomy: {sum(h['holonomy'] for h in history)/len(history):.4f}")

    print(f"\n{'─'*60}")
    print("Final Positions (O=honest, V=drift source):")
    xs, ys = zip(*[(current[n][0], current[n][1]) for n in current])
    rx = max(xs) - min(xs) if max(xs) > min(xs) else 1
    ry = max(ys) - min(ys) if max(ys) > min(ys) else 1
    W, H = 30, 12
    grid = [["." for _ in range(W)] for _ in range(H)]
    for name in current:
        x, y = current[name]
        cx = int((x - min(xs)) / rx * (W - 1))
        cy = int((y - min(ys)) / ry * (H - 1))
        cx, cy = max(0, min(W - 1, cx)), max(0, min(H - 1, cy))
        grid[H - 1 - cy][cx] = "V" if name == violator_actual else "O"
    for row in grid:
        print("  " + " ".join(row))

    return history, current


# ── Main ─────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="Swarm Formation via ZHC")
    p.add_argument("--shape", choices=SHAPES.keys(), default="pentagram")
    p.add_argument("--steps", type=int, default=15)
    p.add_argument("--drift", type=float, default=0.3)
    p.add_argument("--gain", type=float, default=0.6)
    p.add_argument("--tol", type=float, default=0.1)
    p.add_argument("--vi", type=int, default=None, help="Violator robot index")
    args = p.parse_args()
    run_simulation(vi=args.vi, shape=args.shape, steps=args.steps,
                   drift=args.drift, gain=args.gain, tol=args.tol)


if __name__ == "__main__":
    main()
