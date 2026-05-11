#!/usr/bin/env python3
"""
Robot Arm Constraint Planning using Laman Rigidity.

Uses Laman's theorem (E = 2V - 3) to plan minimum-viable robot arm
configurations. Demonstrates under-constrained, just-constrained
(minimally rigid), and over-constrained arm configurations.

A robot arm with V joints needs E = 2V - 3 geometric constraints
for deterministic behavior. Fewer constraints = floppy arm.
Exactly 2V - 3 = minimally rigid (efficient).
More than 2V - 3 = over-constrained (redundant).

Usage:
    python arm.py                          # Full demo with all configurations
    python arm.py --scenario under         # Under-constrained only
    python arm.py --scenario minimal       # Minimally rigid
    python arm.py --scenario redundant     # Over-constrained
    python arm.py --enumerate              # Enumerate configurations for V joints
"""

import sys
import math
import argparse

from fleet_math import ConstraintGraph
from fleet_math.laman import (
    is_rigid,
    is_minimally_rigid,
    rigid_margin,
    count_vertices,
    count_edges,
)


# ── Arm Builder ──────────────────────────────────────────────────────────

def build_arm(joints, constraint_pairs=None):
    """Build a robot arm constraint graph.

    Args:
        joints: List of joint names, e.g. ['J0', 'J1', 'J2']
        constraint_pairs: List of (u, v) constraint edges.
            If None, uses default snake-chain (J0-J1, J1-J2, ...)

    Returns:
        ConstraintGraph
    """
    graph = ConstraintGraph()
    for j in joints:
        graph.add_node(j)

    if constraint_pairs is None:
        # Default: chain constraints (J0-J1, J1-J2, ...)
        constraint_pairs = [(joints[i], joints[i+1]) for i in range(len(joints)-1)]

    for u, v in constraint_pairs:
        graph.add_edge(u, v, weight=1.0)

    return graph


def describe_arm(graph, name):
    """Print detailed info about an arm configuration."""
    V = count_vertices(graph)
    E = count_edges(graph)

    laman_min = 2 * V - 3 if V > 2 else 0
    margin = rigid_margin(graph)
    rigid = is_rigid(graph)
    minimal = is_minimally_rigid(graph)

    print(f"\n{'─'*60}")
    print(f"  {name}")
    print(f"{'─'*60}")
    print(f"  Joints (V):       {V}")
    print(f"  Constraints (E):  {E}")
    print(f"  Laman bound:      2V-3 = {laman_min}")
    print(f"  Margin:           E - (2V-3) = {margin:+d}")
    print(f"  Rigid:            {'✅ YES' if rigid else '❌ NO'}")
    print(f"  Minimally rigid:  {'✅ YES' if minimal else '❌ NO'}")

    if not rigid:
        deficit = laman_min - E
        print(f"  → Under-constrained by {deficit} constraint(s)")
    elif minimal:
        print(f"  → Minimally rigid — just-constrained, efficient")
    else:
        redundancy = E - laman_min
        print(f"  → Over-constrained by {redundancy} constraint(s)")

    print(f"  Nodes: {', '.join(graph.nodes)}")
    print(f"  Edges: {', '.join(f'{u}-{v}' for u,v in graph.edges)}")

    return rigid, minimal


# ── Enumerate Configurations ─────────────────────────────────────────────

def enumerate_configs(V_joints, max_extra=3):
    """Enumerate possible constraint configurations for V joints."""
    print(f"\n{'='*60}")
    print(f"Configuration Enumeration: V={V_joints} joints")
    print(f"{'='*60}")

    laman_min = 2 * V_joints - 3 if V_joints > 2 else 0
    print(f"\nLaman bound: 2V-3 = {laman_min}")
    print(f"All possible constraint counts (E):")
    print(f"  E < {laman_min}:  Under-constrained (floppy)")
    print(f"  E = {laman_min}:  Minimally rigid (efficient)")
    print(f"  E > {laman_min}:  Over-constrained (redundant)")

    # Show specific configurations
    joints = [f"J{i}" for i in range(V_joints)]

    # Under-constrained: chain only
    u_graph = build_arm(joints)
    describe_arm(u_graph, "Under-constrained (chain only)")

    # Minimally rigid: chain + cross-braces to hit exactly 2V-3
    if V_joints >= 3:
        extra_needed = laman_min - (V_joints - 1)  # chain has V-1 edges
        if extra_needed > 0:
            cross_edges = []
            # Add cross-braces between non-adjacent joints
            added = 0
            for i in range(V_joints):
                for j in range(i+2, V_joints):
                    if j != i + 1:  # Not already connected
                        cross_edges.append((joints[i], joints[j]))
                        added += 1
                        if added >= extra_needed:
                            break
                if added >= extra_needed:
                    break
            constraints = [(joints[i], joints[i+1]) for i in range(V_joints-1)]
            constraints.extend(cross_edges)
            m_graph = build_arm(joints, constraints)
            describe_arm(m_graph, "Minimally Rigid (chain + cross-braces)")

    # Over-constrained: chain + many cross-braces
    if V_joints >= 4:
        all_constraints = [(joints[i], joints[i+1]) for i in range(V_joints-1)]
        # Add ALL remaining possible edges
        for i in range(V_joints):
            for j in range(i+2, V_joints):
                if j != i + 1:
                    all_constraints.append((joints[i], joints[j]))
        o_graph = build_arm(joints, all_constraints)
        describe_arm(o_graph, "Over-constrained (chain + all cross-braces)")


# ── Dynamic Arm Simulation ───────────────────────────────────────────────

def simulate_arm_2d(V_joints=6, show_ascii=True):
    """2D visual simulation of a robot arm with varying constraint counts.

    Shows the arm in under-, just-, and over-constrained states using
    2D position simulation.
    """
    joints = [f"J{i}" for i in range(V_joints)]

    # Base positions (linear chain along x-axis)
    base_pos = {f"J{i}": (i * 2.0, 0.0) for i in range(V_joints)}

    print(f"\n{'='*60}")
    print(f"Robot Arm 2D Simulation: V={V_joints} joints")
    print(f"{'='*60}")

    for config_name, extra_edges in [
        ("Under-constrained (chain only)", lambda joints: []),
        ("Minimally rigid (chain + minimal cross)", lambda joints: [
            (joints[0], joints[2]),
            (joints[1], joints[3]),
        ] if len(joints) >= 4 else []),
        ("Over-constrained (chain + all edges)", lambda joints: [
            (joints[i], joints[j])
            for i in range(len(joints))
            for j in range(i+2, len(joints))
        ]),
    ]:
        constraints = [(joints[i], joints[i+1]) for i in range(len(joints)-1)]
        constraints.extend(extra_edges(joints))

        graph = build_arm(joints, constraints)
        V = count_vertices(graph)
        E = count_edges(graph)

        rigid_flag = is_rigid(graph)
        margin = rigid_margin(graph)
        laman_min = 2 * V - 3 if V > 2 else 0

        print(f"\n{'─'*50}")
        print(f"  {config_name}")
        print(f"  V={V}, E={E}, 2V-3={laman_min}, margin={margin}")
        print(f"  Status: ", end="")
        if margin < 0:
            print(f"❌ UNDER-CONSTRAINED (floppy)")
        elif margin == 0:
            print(f"✅ MINIMALLY RIGID (efficient)")
        else:
            print(f"🔁 OVER-CONSTRAINED (redundant)")

        if show_ascii and len(joints) <= 10:
            print(f"\n  Arm visualization:")
            # Simulate positions with some natural droop from gravity
            pos = {}
            for i in range(V_joints):
                x = i * 2.0
                droop = 0.0
                if margin < 0:
                    # Under-constrained: more droop, less predictable
                    droop = -0.3 * (i * (V_joints - i))  # Parabolic droop
                    droop *= (1.0 + 0.5 * abs(margin))
                elif margin == 0:
                    # Minimally rigid: minimal droop
                    droop = -0.1 * i if i > 0 else 0.0
                else:
                    # Over-constrained: very little droop
                    droop = -0.05 * i if i > 0 else 0.0
                pos[f"J{i}"] = (x, droop)

            # ASCII render
            xs = [pos[j][0] for j in joints]
            ys = [pos[j][1] for j in joints]
            min_y = min(ys)
            max_y = max(ys)
            range_y = max_y - min_y if max_y > min_y else 1

            W, H = max(V_joints * 3, 12), 10
            grid = [[" " for _ in range(W)] for _ in range(H)]
            for i, j in enumerate(joints):
                x, y = pos[j]
                cx = int(x / (V_joints * 2.0 + 0.5) * (W - 1))
                cy = int((1 - (y - min_y) / range_y) * (H - 1))
                cx = max(0, min(W - 1, cx))
                cy = max(0, min(H - 1, cy))
                grid[cy][cx] = str(i)

            for row in grid:
                print("  " + "".join(row))

    print(f"\n{'─'*50}")
    print("Laman Summary:")
    print(f"  V joints → 2V-3 constraints needed for rigidity")
    print(f"  Chain-only gives V-1 constraints (always under)")
    print(f"  Each cross-brace adds 1 constraint towards rigidity")


# ── Main ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Robot Arm Laman Planning")
    parser.add_argument("--scenario", choices=["under", "minimal", "redundant", "all"],
                        default="all", help="Which configuration to show")
    parser.add_argument("--joints", type=int, default=6,
                        help="Number of joints (V)")
    parser.add_argument("--enumerate", action="store_true",
                        help="Enumerate configurations for V joints")
    parser.add_argument("--no-visual", action="store_true",
                        help="Skip ASCII visualization")
    args = parser.parse_args()

    joints = [f"J{i}" for i in range(args.joints)]
    laman_min = 2 * args.joints - 3 if args.joints > 2 else 0

    print(f"\n{'#'*70}")
    print(f"# Robot Arm Constraint Planning with Laman Rigidity")
    print(f"# V={args.joints} joints, 2V-3={laman_min} constraints for rigidity")
    print(f"{'#'*70}\n")

    # ── Standard descriptions ──
    if args.scenario in ("under", "all"):
        # Under-constrained: just a chain (V-1 edges)
        u_graph = build_arm(joints)
        describe_arm(u_graph, "Under-Constrained Arm (chain only)")

    if args.scenario in ("minimal", "all") and args.joints >= 3:
        # Minimally rigid: chain + extra cross-braces
        extra_needed = laman_min - (args.joints - 1)
        cross_edges = []
        added = 0
        for i in range(args.joints):
            for j in range(i+2, args.joints):
                cross_edges.append((joints[i], joints[j]))
                added += 1
                if added >= extra_needed:
                    break
            if added >= extra_needed:
                break
        constraints = [(joints[i], joints[i+1]) for i in range(args.joints-1)]
        constraints.extend(cross_edges)
        m_graph = build_arm(joints, constraints)
        describe_arm(m_graph, "Minimally Rigid Arm (just-constrained)")

    if args.scenario in ("redundant", "all") and args.joints >= 3:
        # Over-constrained: chain + all possible cross-braces
        all_constraints = [(joints[i], joints[i+1]) for i in range(args.joints-1)]
        for i in range(args.joints):
            for j in range(i+2, args.joints):
                all_constraints.append((joints[i], joints[j]))
        o_graph = build_arm(joints, all_constraints)
        describe_arm(o_graph, "Over-Constrained Arm (redundant)")

    if args.enumerate:
        enumerate_configs(args.joints)

    if not args.no_visual:
        simulate_arm_2d(args.joints, show_ascii=True)


if __name__ == "__main__":
    main()
