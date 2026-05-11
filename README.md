# fleet-math-demos 🚀

Real-world robotics simulation demos built on [`fleet_math`](https://github.com/SuperInstance/fleet-math-py) — a graph-theoretic toolkit for consensus, rigidity, and emergence detection in multi-agent systems.

## Demos

### 1. Swarm Formation Control (`swarm.py`)

Simulate N robots forming a geometric pattern using **Zero Holonomy Consensus (ZHC)**.

```
                    ⚡ ZHC Detects Drift
  O — O — O          holonomy ≠ 0 on cycles
  |   |   |          containing the violator
  O — ⚠️ — O
  |   |   |
  O — O — O
```

**How it works:**
- Each robot is a node in a `ConstraintGraph`
- Edges carry weights representing formation agreement (distance + bearing)
- ZHC checks whether weight-products around all cycles equal 1
- Non-zero holonomy → a robot has drifted from the formation
- Cycle analysis identifies *which* robot is the violator

**Test scenarios:**
- `python swarm.py --shape pentagon` — 5 robots in pentagon, one drifts
- `python swarm.py --shape grid` — 10 robots in grid, ZHC detects multiple drifters
- `python swarm.py --shape triangle` — 3 robots (Laman-minimal), any drift breaks it

**Sample output:**
```
Step 0: holonomy=0.8200, violator=bot_2 ⚠️  Violation  (confidence=100%)
Step 1: holonomy=0.4500, violator=bot_2 ⚠️  Violation  (confidence=100%)
Step 2: holonomy=0.0800, violator=none ✅ Consensus!  (confidence=0%)
```

### 2. Sensor Fault Detection (`sensors.py`)

N sensors measure the same physical value — ZHC detects which one is faulty.

```
   ┌───┐
   │s0 │─25.1✓
   └───┘
     │                ⚡ Non-zero holonomy
   ┌───┐               on cycles containing s2
   │s1 │─25.3✓
   └───┘
     │
   ┌───┐
   │s2 │─35.0⚠️  ← FAULTY (+20% bias)
   └───┘
```

**How it works:**
- Edge weight = `1 / (1 + |reading_i - reading_j|)` — similarity
- Honest sensors agree → holonomy ≈ 0 around their cycles
- Faulty sensor disagrees → non-zero holonomy on cycles containing it
- The sensor appearing in the most violating cycles is the fault candidate

**Test scenarios:**
- `python sensors.py` — 5 sensors, 1 with +20% bias
- `python sensors.py --scenario dual` — 10 sensors, 2 faulty at different biases
- `python sensors.py --scenario triangle` — 3 sensors (demonstrates why N>3 needed)

**Sample output:**
```
Sensor Readings:
  s0: 25.12
  s1: 25.34
  s2: 35.02 ⚠️
  s3: 24.89
  s4: 25.45

Detected faulty sensor(s): ['s2']
Sensor confidence ranking:
  s2: 100% ⚠️ FAULTY
  s0: 14%
  s1: 14%
  s3: 0%
  s4: 0%
```

### 3. Robot Arm Constraint Planning (`arm.py`)

Use **Laman rigidity** (E = 2V - 3) to plan minimum-viable robot arm configurations.

```
  0—1—2—3—4—5  (under-constrained — floppy)
  |  |  |  |
  0—1—2—3—4—5  (minimally rigid — just-constrained)
  |\/|\/|\/|
  0—1—2—3—4—5  (over-constrained — redundant)
```

**How it works:**
- A 2D arm with V joints has 2V degrees of freedom
- Laman's theorem: a framework is **minimally rigid** with exactly E = 2V - 3 constraints
- Fewer constraints → arm flops (under-constrained)
- Exactly 2V-3 → just-constrained (efficient, deterministic)
- More → over-constrained (redundant actuators, fault-tolerant)

**Test scenarios:**
- `python arm.py` — Full demo (all configurations)
- `python arm.py --joins 8` — Arm with 8 joints
- `python arm.py --enumerate` — Show constraint counts for all configs

**Sample output:**
```
  Under-Constrained Arm (chain only)
  V=6, E=5, 2V-3=9, margin=-4
  → Under-constrained by 4 constraint(s)

  Minimally Rigid Arm (just-constrained)
  V=6, E=9, 2V-3=9, margin=0
  → Minimally rigid — just-constrained, efficient

  Over-Constrained Arm (redundant)
  V=6, E=15, 2V-3=9, margin=+6
  → Over-constrained by 6 constraint(s)
```

## Running

```bash
# Install dependencies
pip install -r requirements.txt

# Run any demo
python swarm.py
python sensors.py
python arm.py
```

Each script supports `--help` for options.

## How fleet_math Powers These Demos

| Concept | fleet_math Module | Used In |
|---------|------------------|---------|
| Zero Holonomy Consensus | `zhc.ConstraintGraph` | swarm.py, sensors.py |
| Cycle detection & analysis | `ConstraintGraph.fundamental_cycles()` | swarm.py, sensors.py |
| Violation localization | `ConstraintGraph.check_consensus()` | swarm.py, sensors.py |
| Laman rigidity check | `laman.is_rigid()` | arm.py |
| Minimal rigidity check | `laman.is_minimally_rigid()` | arm.py |
| Rigidity margin | `laman.rigid_margin()` | arm.py |

## Theory

**Zero Holonomy Consensus (ZHC):** A constraint graph has zero holonomy iff the product of edge weights around every cycle equals 1. Non-zero holonomy indicates constraint violation — used here to detect formation drift and sensor faults.

**Laman's Theorem:** A generic bar-joint framework in 2D is minimally rigid iff it has exactly E = 2V - 3 edges and every subset of k vertices spans at most 2k - 3 edges. This sets the fundamental efficiency bound for robot arm design.
