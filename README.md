# FIRMCP + STL_BOW

Belief-space planning for ground robots, fused with Signal Temporal Logic
(STL) reasoning from
[stl_bow_v2](https://github.com/redwan-newaz/stl_bow_v2).

The repo ships **three planners**, each as a self-contained executable, plus a
single `runner.sh` driver that builds, runs and analyses them.

| Planner          | Binary             | Purpose                                                                 |
| ---------------- | ------------------ | ----------------------------------------------------------------------- |
| FIRM             | `bsp-app-demo`     | Original Feedback Information RoadMap (offline DP over a roadmap).      |
| FIRMCP           | `firmcp-demo`      | FIRM with online POMCP rollout (tree-search over the roadmap).          |
| FIRMCP + STL     | `firmcp-stl-demo`  | FIRMCP whose rollout is biased by STL robustness (two MCTS variants).   |

The third planner (FIRMCP + STL) is the integration point with `stl_bow_v2`:

- targets / landmarks come from a `.spec` file (parameters `gx*, gy*`),
- collision checking uses `stl_bow_v2/lib/collision` (`QuadtreeCollisionChecker`),
- two MCTS rollout strategies are exposed (`stl_guided`, `stl_bow`) so they can
  be benchmarked side-by-side.

---

## Repository Layout

```
firmcp/
├── runner.sh                       # unified launcher (build / run / analyse)
├── examples/stl/                   # bundled benchmark workspace + STL spec
│   ├── bench_small.yaml
│   └── bench_small.spec
├── include/STL/                    # STL integration layer (header)
├── src/STL/                        # STL integration layer (impl)
├── include/Setup/STLFIRMCPSetup.h  # mesh-free, spec-driven FIRMCP setup
├── src/main_firmcp_stl.cpp         # benchmark driver
├── src/main_firmcp.cpp             # original FIRMCP demo
├── src/main_firm.cpp               # original FIRM demo
├── SetupFiles/                     # XML configs for FIRM / FIRMCP demos
├── Models/                         # mesh files for FIRM / FIRMCP demos
└── Results/firmcp_stl-<TS>/        # per-run output (created on demand)
```

---

## Dependencies

System packages (Ubuntu 22.04 reference):

- C++17 toolchain, CMake ≥ 3.20, `pkg-config`
- Boost (`date_time thread serialization filesystem system program_options unit_test_framework chrono`)
- OMPL (`libompl-dev`)
- ompl_app (for `RigidBodyGeometry`, used by the original FIRM/FIRMCP demos)
- Armadillo (`libarmadillo-dev`)
- TinyXML (`libtinyxml-dev`)
- SQLite3 (`libsqlite3-dev`)
- FCL + libccd (`libfcl-dev libccd-dev`) — original demos only
- Assimp (`libassimp-dev`)                  — original demos only
- Eigen3 (`libeigen3-dev`)
- yaml-cpp (`libyaml-cpp-dev`)
- `sqlite3` CLI (used by `runner.sh analyze`)

External project (must be built first):

- [`stl_bow_v2`](https://github.com/redwan-newaz/stl_bow_v2) at
  `/home/redwan/CppDev/stl_bow_v2` — provides the collision library
  (`lib/collision/lib/lib*.so`) and `libstlrom.a` (built into
  `stl_bow_v2/build/`). Override the path with
  `-DSTL_BOW_ROOT=/path/to/stl_bow_v2`.

---

## Build

One-liner:

```bash
./runner.sh build              # builds bsp-app-demo, firmcp-demo, firmcp-stl-demo
```

Manual cmake invocation:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target firmcp-stl-demo -j$(nproc)
```

A successful build drops the binaries at the repo root (`./bsp-app-demo`,
`./firmcp-demo`, `./firmcp-stl-demo`).

---

## Run

### FIRMCP + STL benchmark (recommended entry point)

Two modes:

```bash
# Smoke test (default) — only builds the roadmap, no executor.
# Fast (~10s/variant), but the two MCTS variants will look identical because
# they only differ during rollout.
./runner.sh
./runner.sh stl <yaml> <spec> <plan_seconds>

# Full benchmark — runs executeFeedbackWithPOMCP() for max_steps per variant.
# This is the only mode that exercises the rollout-policy difference.
./runner.sh full
./runner.sh stl <yaml> <spec> <plan_seconds> <max_exec_steps>
```

`max_exec_steps` is a hard cap on the simulated steps the POMCP executor is
allowed to take. The executor has no built-in goal-progress test, so without a
cap it can run for many minutes per variant. Sensible defaults:

| `max_exec_steps` | Behaviour                                         |
| ---------------- | ------------------------------------------------- |
| 0 (default)      | Roadmap-only smoke test. ~10 s/variant.           |
| 30               | Quick rollout sample. ~30 s/variant.              |
| 100              | `full` default. ~2 min/variant on `bench_small`.  |
| 200+             | Long horizon, useful for goal-reach experiments.  |

Each run produces:

```
Results/firmcp_stl-<TIMESTAMP>/
├── README.txt                           # input metadata + max_exec_steps
├── benchmark.csv                        # variant,plan_seconds,solved,
│                                        # exec_seconds,exec_steps,
│                                        # reached_goal,rob_phi,rob_reach,rob_safe
├── stl_guided/
│   ├── setup_bridge.xml                 # synthesised motion-model + FIRM/FIRMCP params
│   ├── summary.txt
│   └── run-<TIMESTAMP>/
│       ├── results.db                   # SQLite (roadmap + execution traces)
│       ├── FIRMRoadMap-*.xml            # only emitted in smoke mode (max_exec_steps=0)
│       ├── DijkstraSolveTime.txt
│       ├── DPSolveTime.txt
│       └── StartStateAddTime.txt
└── stl_bow/    (same layout)
```

The roadmap XML is intentionally **not** dumped after a full-mode run because
POMCP grows the in-memory graph with belief-tree vertices and serialising it
can OOM. The SQLite `roadmap_*` tables still hold the static planning
roadmap.

### Original FIRM / FIRMCP demos

```bash
./runner.sh firm   SetupFiles/SetupFIRMCP.xml
./runner.sh firmcp SetupFiles/SetupFIRMCP.xml
```

---

## Benchmark

`firmcp-stl-demo` runs both MCTS variants back-to-back on the same workspace
+ spec, using the same node-count budget. The two variants share everything
except the rollout edge-selection rule:

| Variant      | Rule                                                                        |
| ------------ | --------------------------------------------------------------------------- |
| `stl_guided` | `cost'(e) = FIRMCP_cost(e) − α · STL_rob(traj_to_target)`                   |
| `stl_bow`    | `score(e)  = STL_rob(traj_to_target) − β · FIRMCP_cost(e) + c·UCB(visits)`  |

Both variants build a roadmap, solve Dijkstra + dynamic programming, and emit
the same artefact set. Differences surface during the rollout phase, which is
exercised by `executeFeedbackWithPOMCP` (called from the `runExecution`
hook). The bundled benchmark currently stops at `solve()` — see
[Limitations](#limitations).

A typical run on `examples/stl/bench_small.{yaml,spec}` (1 target, 0
spec-obstacles, 15 s budget):

| Metric                  | `stl_guided` | `stl_bow` |
| ----------------------- | ------------ | --------- |
| Plan time               | 8.84 s       | 9.44 s    |
| Roadmap nodes           | 251          | 251       |
| Roadmap edges           | 7712         | 8160      |
| Avg edge cost           | 151.2        | 140.8     |
| Avg edge `success_prob` | 0.557        | 0.576     |
| Dijkstra / DP solve     | 3 / 6 ms     | 2 / 3 ms  |

To compare more workspaces, drop additional `.yaml` + `.spec` pairs under
`examples/stl/` and feed them to `./runner.sh stl …`.

---

## Analyse

```bash
./runner.sh latest                                    # most recent STL run
./runner.sh analyze Results/firmcp_stl-20260510T173922
```

The analyser prints, per variant:

- variant summary (yaml/spec/plan time/solved/STL robustness),
- roadmap location + node/edge counts,
- Dijkstra + DP solve times,
- SQLite table sizes,
- edge cost and success-probability statistics (min / avg / max).

Inspecting the SQLite DB directly:

```bash
sqlite3 Results/firmcp_stl-*/stl_bow/run-*/results.db ".tables"
# bounds  goal_state  landmarks  metadata  roadmap_edges  roadmap_nodes  start_state
```

Tables follow `include/Planner/FIRM.h::saveSetupAndGraphToDB()`:

| Table           | Columns                                                              |
| --------------- | -------------------------------------------------------------------- |
| `metadata`      | `key, value`                                                         |
| `bounds`        | `low_x, high_x, low_y, high_y`                                       |
| `landmarks`     | `id, x, y`                                                           |
| `start_state`   | `x, y, theta`                                                        |
| `goal_state`    | `x, y, theta`                                                        |
| `roadmap_nodes` | `id, x, y, theta, cov_xx, cov_xy, cov_yy, is_start, is_goal`         |
| `roadmap_edges` | `source, target, cost, success_prob`                                 |

---

## Planners — benefits & limitations

### 1. `bsp-app-demo` (FIRM)

**What it does.** Builds a probabilistic roadmap in belief space, runs an
offline Dynamic Program over the graph, then drives the robot using the
resulting feedback policy.

**Benefits**
- Cheap online cost: feedback policy is precomputed; execution is a graph
  lookup.
- Provably converges to a globally consistent value function over the
  roadmap.
- Multi-query reuse: the same roadmap solves many start/goal pairs.

**Limitations**
- Single, fixed graph — quality is bounded by sampling density.
- Cannot react to deviations not anticipated by the offline DP.
- Heavy dependency surface (FCL meshes, ompl_app `RigidBodyGeometry`).

### 2. `firmcp-demo` (FIRMCP)

**What it does.** Adds POMCP-style online tree search on top of the FIRM
graph; rollouts simulate edge execution before committing to an action.

**Benefits**
- Reacts online to belief evolution: better than offline FIRM when the actual
  trajectory drifts from the precomputed policy.
- Reuses FIRM's roadmap and value function as the rollout fallback (warm
  start).
- Anytime: more particles → better decisions, but always returns *something*.

**Limitations**
- Per-step planning cost is significant (`numPOMCPParticles × maxPOMCPDepth`
  rollouts).
- Inherits FIRM's mesh / FCL dependencies.
- Tuning is finicky: `cExplorationForSimulate`, `cExploitationForRollout*`,
  `nSigmaForPOMCPParticle`, etc. all interact.

### 3. `firmcp-stl-demo` — variant `stl_guided`

**What it does.** Same FIRMCP machinery, but the rollout edge cost is shaped
by a linear penalty on STL robustness:

```
cost'(edge) = FIRMCP_cost(edge) − α · STL_robustness(line(current, target))
```

**Benefits**
- Drop-in: just one virtual override, no change to POMCP plumbing.
- Steers exploration toward edges that satisfy more of the spec.
- The STL signal is interpretable — easy to tune `α` based on cost vs
  robustness scales.
- Uses the spec as the source of truth for *what* the robot must do (visit
  ordered targets, avoid obstacles), not just *where* the goal is.

**Limitations**
- Linear shaping is myopic: it ranks each edge in isolation, ignoring how the
  whole trajectory composes.
- Sensitive to `α`. Too small → no effect; too large → ignores collision
  cost.
- The robustness probe is computed on a *straight line* between the current
  and target nodes, not on the simulated edge controller's actual rollout.

### 4. `firmcp-stl-demo` — variant `stl_bow`

**What it does.** Same hook, but uses a STLBOW-style acquisition that
combines spec satisfaction, FIRMCP cost, and a UCB exploration bonus over
per-edge visit counts:

```
score(edge) = STL_robustness − β · FIRMCP_cost + c · √(log(N+1)/(n+1))
```

**Benefits**
- Active exploration: the UCB term forces the planner to try under-visited
  edges before committing.
- Closer in spirit to the `stl_bow_v2/KinoBOW` Bayesian-optimisation rollout.
- Tends to discover lower-cost paths over multi-step planning (avg edge cost
  140.8 vs 151.2 in the bundled benchmark — see
  [caveat](#limitations)).

**Limitations**
- The visit-count tracker uses thread-local state; not safe across parallel
  planner instances.
- `β` and `c` need joint tuning; bad values either collapse to greedy or
  thrash on exploration.
- Same straight-line robustness probe limitation as `stl_guided`.

---

## Limitations

These are **known issues** in the current integration. None of them block
running the benchmark, but they shape how the numbers should be interpreted.

1. **Robustness probes a straight-line proxy, not the executed trajectory.**
   `benchmark.csv` reports `rob_phi`, `rob_reach`, `rob_safe` for the
   straight-line path through the spec's ordered targets, evaluated against
   the spec with the `ox` channel populated by nearest-obstacle distance
   (from `SpecLandmarks::obstacles()`). This is variant-independent — both
   `stl_guided` and `stl_bow` get the same score because they share the
   same target list. To compare actual *executed* trajectories, replay them
   from the `cost_history` table in `results.db` instead.
2. **Default mode is roadmap-only.** With `max_exec_steps=0` the driver
   stops after FIRMCP builds the roadmap and runs DP. The two MCTS variants
   only differ during `executeFeedbackWithPOMCP`, so any edge-count / cost
   delta in this mode is RNG variance, not algorithm signal. Use
   `./runner.sh full` (or pass a `max_exec_steps>0` argument to `stl`) to
   actually exercise the rollout policies.
3. **Original FIRM/FIRMCP demos still need meshes.** Only the new
   `firmcp-stl-demo` is mesh-free (uses `QuadtreeCollisionChecker` from
   `stl_bow_v2`). The other two still depend on `Models/*.obj` via FCL.
4. **STL spec parser is a subset.** `SpecLandmarks` recognises
   `param`-style `gx*/gy*` / `ox*_x/oy*_y` declarations and ignores the
   formula structure. Anything outside the `param` block is opaque to the
   parser.
5. **Rollout-policy STL probe uses straight-line interpolation.** A more
   faithful evaluation would simulate the FIRMCP edge controller and score
   the *realised* trajectory, at the cost of one full rollout per scored
   edge.
6. **`STL_BOW_ROOT` is a hard-coded default.** CMake defaults to
   `/home/redwan/CppDev/stl_bow_v2`; override with
   `-DSTL_BOW_ROOT=/path/to/stl_bow_v2` if your checkout lives elsewhere.

---

## Adding new benchmarks

1. Drop a new `.yaml` + `.spec` pair into `examples/stl/`. The yaml needs at
   least `max_speed`, `min_speed`, `max_yawrate`, `dt`, `predict_time`,
   `goal_radius`, `boundary` (`[xmin, xmax, ymin, ymax]`), and `obstacles`
   (list of `[x, y]`). The spec needs a `param` line declaring `rx, ry,
   rtheta` (start) and one or more `gx<i>, gy<i>` pairs (targets).
2. Run `./runner.sh stl examples/stl/<yaml> examples/stl/<spec> <seconds>`.
3. `./runner.sh latest` to inspect.

---

## License

Mirror of the upstream FIRM repository's BSD license; see
`include/**/*.h` headers for the per-file notice.
