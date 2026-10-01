# Path Planning with Dynamic Programming

**See how a lateral path emerges from a layered search graph.** This small MATLAB example searches a collision-checked path through a road corridor containing static circular obstacles. It exposes the lattice construction, transition costs, parent selection and backtracking in readable functions.

The planner operates on **station `s` and lateral offset `l`**. It chooses the geometric path; it does not optimize a time-dependent speed profile. The companion [SpeedDeciderViaDP](https://github.com/libai1943/SpeedDeciderViaDP) illustrates that separate planning problem.

## Run

```matlab
cd('C:/path/to/PathDeciderViaDP');
RunMe;
```

The visible implementation uses standard MATLAB numerical, polygon and plotting functions. It does not call AMPL, IPOPT, CasADi or a separate optimization toolbox. Keep the files together and run from the repository root so the helper functions are found.

`RunMe.m` initializes the vehicle and lattice, defines four static obstacles, calls `PathPlanningViaDP`, and plots the resulting path with vehicle footprints. The searched arrays are available as `s`, `l`, and in global `params_.s`, `params_.l`.

## How it works

```mermaid
flowchart LR
    A[Road bounds and circular obstacles] --> B[Station / lateral lattice]
    B --> C[Candidate parent-to-child connections]
    C --> D[Derivative limits and sampled collision checks]
    D --> E[Accumulate cost and store best parent]
    E --> F[Backtrack the selected path]
    F --> G[Path and vehicle-footprint plot]
```

The lattice advances through station layers. Candidate lateral positions are distributed between the road bounds at each layer. An edge cost combines lateral offset and finite-difference approximations to the first, second and third derivatives of offset with respect to station. Invalid derivative values or detected collisions receive a large infeasibility cost.

Each node retains a best accumulated cost, its parent and derivative information. Backtracking recovers the selected path. This is a finite-resolution educational search: it should not be described as a certificate of global optimality for a continuous vehicle-motion problem.

Collision checking samples points along each connection. It tests obstacle/road-boundary samples against the rectangular vehicle footprint using `inpolygon`; this is the implementation's geometric approximation, not an exact continuous swept-volume test.

## Functions

| File | Purpose |
| --- | --- |
| `RunMe.m` | Entry script, initial lateral state, obstacle definitions and final plot. |
| `InitializeParams.m` | Vehicle dimensions, planning extent, lattice resolution, derivative limits and cost weights. |
| `PathPlanningViaDP.m` | Layered dynamic-programming search and path recovery. |
| `GetStatesOfCurrentNode.m` | Recover the station derivatives of a candidate transition. |
| `CalculateCost.m` | Transition cost, physical/derivative checks and infeasibility penalties. |
| `IsCurNodeCollidedToObs.m` | Sample a connection and check vehicle-footprint overlap with scene samples. |
| `GetRoadLeftBound.m`, `GetRoadRightBound.m` | Define the road corridor as functions of station. |
| `GenerateCircularObsBarrier.m` | Sample circular obstacle boundaries. |
| `CreateVehiclePolygon.m` | Construct a rectangular vehicle footprint. |
| `ResampleProfiles.m` | Resample profiles for downstream display/use. |
| `asd.m` | Draw the road/lattice background used during search visualization. |
| `dsa.m` | Draw the searched path and vehicle footprints. |

The two short plotting filenames are retained to match the original code. They are callable helpers, not additional planning algorithms.

## Main parameters

| Setting in `InitializeParams.m` | Default |
| --- | --- |
| Station extent | 120 m |
| Station layers / lateral samples | 30 / 10 |
| Station step | 4 m |
| Collision-check resampling distance | 0.2 m |
| Bounds on `dl/ds` and `d²l/ds²` | 3 and 10 |
| Weights for `l`, `dl`, `ddl`, `dddl` | 1 each |
| Wheelbase / width | 2.8 / 1.942 m |

The variables called `time_horizon` and `v_max` determine the station extent in this path demo; they do not make the result a timed trajectory. Set the initial lateral state and circular obstacles in `RunMe.m`, and edit the two road-bound functions to change the corridor.

To hide the valid-edge display, set `params_.dp.utility.enable_plot_valid_connections_between_adjacent_layers = 0` in `InitializeParams.m`. The final path/footprint plot is a separate call to `dsa`.

## Acknowledging the implementation

To reference this exact demonstration, cite the repository and include the commit used in your experiment:

```bibtex
@misc{LiPathDeciderViaDP,
  author = {Li, Bai},
  title = {{PathDeciderViaDP}: MATLAB Path Planning with Dynamic Programming},
  howpublished = {GitHub repository},
  url = {https://github.com/libai1943/PathDeciderViaDP}
}
```

The distributed source does not identify a specific source article. The software citation above identifies the actual implementation without attributing it to an unverified paper. For a complete search-and-optimization research planner by the author, see [OnRoadPlanner_IFAC2020](https://github.com/libai1943/OnRoadPlanner_IFAC2020) and its own paper citation.
