# JRC-FSM with Perception Latency and Errors

> **NOTICE OF MODIFICATION** — This is a **Derivative Work** based on
> **JRC-FSM**, Copyright © 2021 European Union, licensed under the
> **European Union Public Licence (EUPL) v. 1.2**.
> The Work **has been modified**. Modifications were made between
> **1 July 2026 and 19 July 2026**, and are described under
> [Modifications from Upstream](#modifications-from-upstream).
> This Derivative Work is distributed under the **same EUPL v. 1.2 licence**;
> a copy is included as `EUPL-1.2 EN.txt`.

A fork of the European Commission JRC's [JRC-FSM](https://github.com/ec-jrc/JRC-FSM)
reference implementation of the UNECE R157 safety models, extended so the ego's
**perception of the other vehicle can be degraded** — with Gaussian noise, with
sensing latency, or both — instead of being perfect.

Upstream, every model reasons on the exact state of the conflict vehicle. That
makes the resulting crash boundaries a property of the *decision logic* alone.
This fork inserts a perception layer between the world and the model, so the same
boundaries can be measured under realistic sensing error. The true vehicle state
is never modified — it still drives the other vehicle's motion and the crash
test — so the perturbation isolates perception error from the underlying
dynamics.

With every knob left at its default of `0.0`, the perception layer is skipped
entirely and the fork reproduces upstream behaviour exactly.

---

This document describes how to use the following repository to 
investigate the behavior of the proposed safety models for UNECE R157.

Three reference scenarios are implemented in the current formulation:

1. 'cut_in';
2. 'cut_out';
3. 'car_following'.

Four models are here discussed:

1. 'FSM' : Fuzzy Safety Model [1];
2. 'RSS' : Responsibility Sensitive Safety [2];
3. 'CC_human_driver' : [3];
4. 'Reg157' : [4];

The python script 'safety_check_runner.py' provides the possibility of selecting
three types of analyses: 

1. 'one_case' : enables selecting one concrete scenario, one model, and 
   visually inspecting the result of the simulation;
2. 'comparison' : aims at a systematic investigation of the safety models 
   on a selection of logical scenario and then stores data for later processing;
3. 'post_processing' : provides the possibility of visually inspect the results of 
    the previously executed 'comparison' scenario.

The 'safety_check_runner.py' can be launched either via command line providing 
the minimum number of arguments or called from another python script as shown 
in the 'example.py' file.


### One Case
The 'one_case' analysis is a single simulation which involves a concrete scenario with 
a specified safety model. 

An example command line instruction to execute a 'cut_in' scenario is:

`python safety_check_runner.py one_case cut_in` 

The default model for the 'one_case' analysis is FMS, however other models can be 
executed too as:

`python safety_check_runner.py one_case cut_in model=RSS`

The concrete scenarios can be modified via passing optional parameters.<br> 
In particular, the following parameters can be adjusted on the fly:
- the *initial velocity* of the simulation can be set via passing `initial_speed` optional 
command in (km/h):

`python safety_check_runner.py one_case cut_in model=RSS initial_speed=50`

- the leader *deceleration* of car-following scenario can be adjusted (in m/s<sup>2</sup>) as: 

`python safety_check_runner.py one_case car_following model=Reg157 deceleration=3`

- the *obstacle speed* in the cut-in scenario can be updated (in km/h, default=) via:

`python safety_check_runner.py one_case cut_in model=CC_human_driver obstacle_speed=20`

- the *lateral speed* of cut-out and cut-in maneuvers can be adjusted (in m/s, default=-1.) as:

`python safety_check_runner.py one_case cut_in model=CC_human_driver lateral_speed=-2.`

- the *front distance* of cut-out and cut-in maneuvers can be adjusted (in m, default=50) as:

`python safety_check_runner.py one_case cut_in model=FSM front_distance=30`    

### Comparison

The comparison cases loop over the safety models for a range concrete scenarios belonging to 
the same class.

Three alternatives are possible corresponding to the 

- cut-in scenario:

`python safety_check_runner.py comparison cut_in`

- cut-out scenario

`python safety_check_runner.py comparison cut_out`

- car-following scenario

`python safety_check_runner.py comparison car_following`

Unlike upstream, the cut-in comparison accepts optional parameters to restrict the
sweep to a single operating point, so one published example can be reproduced
without re-running the whole grid. All accept comma-separated lists; omitting them
runs the full sweep exactly as before.

```bash
# one cell: ego 60 km/h, cut-in vehicle 50 km/h, all four models
python safety_check_runner.py comparison cut_in ego_speed=60 obstacle_speed=50 speed_range=low

# restrict to a single model as well
python safety_check_runner.py comparison cut_in ego_speed=60 obstacle_speed=50 model=CC_human_driver
```

| keyword | meaning |
|---|---|
| `ego_speed` | ego speeds to sweep (km/h) |
| `obstacle_speed` | cut-in vehicle speeds (km/h); `cut_in_speed` also accepted |
| `model` | restrict to a subset of `FSM`, `CC_human_driver`, `RSS`, `Reg157` |
| `speed_range` | `low`, `high`, or both (default) |

A restricted run writes byte-identical files to what the full sweep would have
produced for those cells — the grids, ordering and per-cell noise seeds are
unchanged.

The perception knobs above apply to `comparison` runs too.

### Post processing

Eventually, a post processing analysis can be run to save/visualize the results 
of the comparisons.

- cut-in post-processing:

`python safety_check_runner.py post_processing cut_in`

    - a detailed analysis on TTC is possible for the cut_in case by passing a safety model specification  
     
        `python safety_check_runner.py post_processing cut_in model=RSS` 

    - a detailed analysis on FSM criticality is possible for the cut_in case by passing the FSM model specification  
     
        `python safety_check_runner.py post_processing cut_in model=FSM` 

- a **single-model crash map** for the cut-in case, plotting every swept cell as
  crash or no-crash rather than overlaying four models' crash cells:

    `python safety_check_runner.py post_processing cut_in only_model=CC_human_driver save_image=True`

    Accepts the same `ego_speed` / `obstacle_speed` restriction as `comparison`.
    Marker shape carries the outcome alongside colour, so the figure stays
    readable under red-green colour vision deficiency.

- cut-out scenario

`python safety_check_runner.py post_processing cut_out`

- car-following scenario

`python safety_check_runner.py post_processing car_following`

Finally, a 'save_image' boolean can be passed to automatically store the image on the 
hard drive instead of the graphical visualization only.

`python safety_check_runner.py post_processing car_following save_image=True`

## Perception Error and Latency

The perception layer is configured through `utility/global_parameters.py`, or
overridden per-run from the command line. Both effects apply to the ego's view of
the conflict vehicle only.

| parameter | CLI keyword | default | meaning |
|---|---|---|---|
| `perception_noise_sigma_pos` | `noise_pos` | `0.0` | std-dev of position noise [m], longitudinal and lateral |
| `perception_noise_sigma_speed` | `noise_speed` | `0.0` | std-dev of speed noise [m/s], longitudinal and lateral |
| `perception_noise_seed` | `noise_seed` | `12345` | base RNG seed, for reproducibility |
| `perception_latency_mean_s` | `latency_mean` | `0.0` | mean sensing latency lambda [s] |
| `perception_latency_mode` | `latency_mode` | `constant` | how the per-step delay is drawn |

**Noise** perturbs the perceived position and speed by independent
`N(0, sigma^2)` draws at every timestep.

**Latency** makes the ego perceive the other vehicle's state from `t - lambda`.
The delay is converted to whole simulation steps, drawn according to
`latency_mode`:

- `constant` — a fixed delay at every step
- `fixed_poisson` — one Poisson draw per scenario, then held constant
- `poisson` — a fresh Poisson draw at every step (jittery)

Noise is seeded from the *scenario parameters* rather than the model, so every
model sees the **same noise realisation** on the same scenario cell. Model
comparisons therefore remain paired and stay meaningful under perturbation.

### Running with perturbed perception

```bash
# Gaussian position noise, sigma = 0.5 m
python safety_check_runner.py comparison cut_in noise_pos=0.5

# 0.3 s Poisson sensing latency, no noise
python safety_check_runner.py comparison cut_in latency_mean=0.3 latency_mode=poisson

# both, with an explicit seed
python safety_check_runner.py comparison cut_in \
    noise_pos=0.5 noise_speed=1.0 latency_mean=0.15 latency_mode=poisson noise_seed=7
```

### Where results are written

Perturbed runs write to their own directories rather than overwriting the clean
baseline. The suffix encodes the active error level:

```
results/cut_in_low_speed/                                  # clean, no perturbation
results/cut_in_low_speed_noise_p0.5_s1/                    # noise only
results/cut_in_low_speed_noise_lat0.3poisson/              # latency only
results/cut_in_low_speed_noise_p0.5_s1_lat0.15poisson/     # both
```

### Additional output columns

The `comparison` CSVs carry the perceived **and** true state of the conflict
vehicle, so perception error can be quantified per cell rather than inferred:

| group | columns |
|---|---|
| closest approach | `perceived_gap` / `true_gap`, `perceived_lat_gap` / `true_lat_gap`, `perceived_speed` / `true_speed`, `perceived_lat_speed` / `true_lat_speed` |
| lane entry | the same eight, prefixed `entry_` — sampled at the cut-in's lane-entry instant; `NaN` if the actor never entered the ego lane |
| context | `ego_speed`, `entry_ego_speed`, `reacted` (did the ego ever brake) |

These are filled for every cell, not only the ones where the ego braked. With
perception disabled, each perceived column equals its true counterpart.

## Dependencies
The following python packages are required to run the models:
- pandas (New BSD License, 3-clause);
- numpy (New BSD License, 3-clause);
- numba (BSD 2-clause);
- scipy (BSD);
- matplotlib (BSD).


## Repository Layout

```
safety_check_runner.py      # entry point: one_case / comparison / post_processing
example.py                  # calling the runner from Python instead of the CLI
jrc_sim.py                  # standalone closed-form CC crash band + cut-in sim,
                            #   used by external tooling without the full loop
utility/
├── global_parameters.py    # vehicle/sim constants + the perception knobs
├── movement.py             # motion integration, PerceptionNoise, crash test
├── models.py               # FSM / RSS / Reg157 / CC_human_driver checks
├── comparison_*.py         # the systematic sweeps, one per scenario family
└── one_case_*.py           # single-scenario runs
post_processing/            # figures from stored comparison results
AIAwarePCBoutput/           # stored sweeps at various perception-error levels
├── no_error/               #   clean baseline
├── perror{0.15..1.2}/      #   position-noise sweeps
├── no_poisson_{0.3,0.6,1}/ #   constant-latency variants
└── csv/                    #   flattened per-level summaries
```

## Licence and Copyright

This repository is a **Derivative Work** of **JRC-FSM**, produced by the Joint
Research Centre of the European Commission.

> Original Work: **JRC-FSM** — https://github.com/ec-jrc/JRC-FSM
> Copyright © **2021 European Union**
> Licensed under the **EUPL v. 1.2**

Licensed under the EUPL, Version 1.2 or – as soon as they will be approved by
the European Commission – subsequent versions of the EUPL (the "Licence"). You
may not use this work except in compliance with the Licence. You may obtain a
copy of the Licence at:

https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12

Unless required by applicable law or agreed to in writing, software distributed
under the Licence is distributed on an **"AS IS" basis, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND**, either express or implied. See the Licence for the
specific language governing permissions and limitations under the Licence.

### Compliance with EUPL-1.2 Article 5

The EUPL is a **copyleft** licence. Article 5 imposes three obligations on any
Derivative Work, addressed as follows:

| Obligation (Art. 5) | How this repository complies |
|---|---|
| *Attribution right* — keep intact all copyright and licence notices | The `Copyright (c) 2021 European Union` and EUPL headers are retained verbatim at the top of **every** source file, including files added by this fork |
| *Attribution right* — include a copy of the notices **and of the Licence** with every copy distributed | The full licence text ships as `EUPL-1.2 EN.txt` |
| *Attribution right* — Derivative Works must carry **prominent notices stating that the Work has been modified and the date of modification** | The notice at the top of this README, plus the dated summary below |
| *Copyleft clause* — Distribution of a Derivative Work must be under the EUPL (or a later version) | This repository is distributed under EUPL v. 1.2; no additional or restrictive terms are imposed |

### Modifications from Upstream

Modified between **1 July 2026 and 19 July 2026** by **TEJAPIJAYA Thanawat**.
The four R157 safety models (`FSM`, `RSS`, `Reg157`, `CC_human_driver`) are
**unchanged**; the modifications add a perception layer around them and the
instrumentation needed to measure its effect:

1. **Perception error and latency layer** (`utility/global_parameters.py`,
   `utility/movement.py`) — Gaussian position/speed noise and constant or
   Poisson sensing latency applied to the ego's view of the conflict vehicle,
   with a `PerceptionNoise` class and seeded, reproducible draws. Disabled by
   default.
2. **Perceived-vs-true instrumentation** (`utility/comparison_*.py`) — the
   comparison CSVs gained paired perceived/true columns at closest approach and
   at lane entry, so perception error can be quantified per scenario cell.
3. **Perturbation-aware output paths** (`utility/movement.py: noise_suffix`) —
   perturbed runs write to suffixed directories rather than overwriting clean
   baselines.
4. **Perception overrides on the runner** (`safety_check_runner.py`) —
   `noise_pos`, `noise_speed`, `noise_seed`, `latency_mean`, `latency_mode`.
5. **Operating-point restriction of the cut-in sweep**
   (`utility/comparison_cut_in.py`, `safety_check_runner.py`) — `ego_speed`,
   `obstacle_speed`, `model`, `speed_range`.
6. **Single-model crash map** (`post_processing/`) — `only_model=` rendering of
   every swept cell as crash / no-crash.
7. **`jrc_sim.py`** — a standalone closed-form CC crash band and cut-in
   simulation, callable without the full comparison loop.
8. **`AIAwarePCBoutput/`** — stored sweep results at a range of perception-error
   levels.

## Citation

If you use this repository, please cite **both** the original JRC work and this
extension. The safety models are the JRC's; only the perception layer is added
here.

```bibtex
@software{jrc_fsm,
  title  = {JRC-FSM: Reference implementation of UNECE R157 safety models},
  author = {{European Commission, Joint Research Centre}},
  year   = {2021},
  url    = {https://github.com/ec-jrc/JRC-FSM},
  note   = {Licensed under EUPL-1.2}
}

@software{jrc_fsm_perception_latency_errors,
  title  = {JRC-FSM with Perception Latency and Errors},
  author = {TEJAPIJAYA, Thanawat},
  year   = {2026},
  url    = {https://github.com/thanawat-tej/jrc-fsm-perception-latency-errors},
  note   = {Derivative Work of JRC-FSM (European Union, 2021), EUPL-1.2}
}
```

Please also cite the underlying model papers listed under
[References](#references).

## References
[1] Mattas, Konstantinos, et al. "Fuzzy Surrogate Safety Metrics for real-time assessment of rear-end collision risk. 
    A study based on empirical observations." Accident Analysis & Prevention 148 (2020): 105794.

[2] Shalev-Shwartz, Shai, Shaked Shammah, and Amnon Shashua. 
    "On a formal model of safe and scalable self-driving cars." 
    arXiv preprint arXiv:1708.06374 (2017).

[3] UNECE Reg 157 Annex 4 - Appendix 3

[4] UNECE Reg 157 Paragraph 5.2.5.2.