# ADP-Based Fault-Tolerant Control for Nonlinear Systems

MATLAB implementation of an observer-based fault-tolerant controller for a continuous-time nonlinear system with simultaneous additive actuator and sensor faults. The workflow combines sensor-fault decoupling, adaptive state and actuator-fault estimation, algebraic sensor-fault reconstruction, and an offline approximate dynamic programming (ADP) controller.

The supplied example is a two-state demonstration system. The nominal policy is learned from the known plant model using a single polynomial critic and least-squares policy iteration, then held fixed during closed-loop simulation.

## Method

The plant and measurement models are

```math
\dot{x}=Ax+Bu+g(x)+D_a f_a
```

```math
y=Cx+D_s f_s
```

### 1. Sensor-fault decoupling

Form the virtual output $`y_0=N_s y=C_0x`$, where $`N_sD_s=0`$ and $`C_0=N_sC`$.

### 2. State and actuator-fault estimation

Use the adaptive observer with innovation $`r=y_0-C_0\hat{x}`$:

```math
\dot{\hat{x}}=A\hat{x}+Bu+g(\hat{x})+D_a\hat{f}_a+Lr
```

```math
\dot{\hat{f}}_a=-\delta\Gamma\hat{f}_a+\Gamma Kr
```

Observer design enforces a Lipschitz-based linear matrix inequality (LMI) together with $`KC_0=D_a^TP`$. The resulting matrices undergo numerical feasibility checks.

### 3. Offline nominal-controller training

Approximate the value function as $`\hat{V}(x)=W^T\sigma(x)`$ and derive a candidate policy from its gradient:

```math
\mu_{\mathrm{cand}}(x)=-\frac{1}{2}R^{-1}B^T\nabla\hat{V}(x)
```

Training uses eight normalized quadratic and quartic monomials on a $`17\times17`$ state grid, excluding the origin: 288 collocation points. Each iteration solves an unregularized least-squares Bellman regression and records its rank, singular values, condition number, and residual. Candidate policies are screened using a quadratic Lyapunov function on the design grid. If necessary, the candidate is blended with the previous policy; if no tested blend passes, the previous policy is retained.

### 4. Actuator-fault compensation

Evaluate the final accepted policy at the estimated state:

```math
u=\mu_N(\hat{x})-B^\dagger D_a\hat{f}_a
```

### 5. Sensor-fault reconstruction

Compute

```math
\hat{f}_s=D_s^\dagger(y-C\hat{x})
```

This reconstruction supports monitoring. Sensor-fault decoupling in the feedback path is provided by the virtual output.

The critic weights remain fixed during simulation. Online adaptation is confined to the observer's actuator-fault estimate.

## Demonstration system

```matlab
A  = [0 1; -2 -0.8];
B  = [0; 1];
C  = [1 0; 0 1; 1 1];
Da = [0; 1];
Ds = [0; 0; 1];
Ns = [1 0 0; 0 1 0];
g  = @(x) [0; 0.25*sin(x(1))];
```

There are two states, one control input, three measured outputs, and one fault channel each for the actuator and sensor. The actuator fault is matched to the input channel: $`D_a=B`$. The sensor fault affects only the third output.

For this example, $`C_0=I_2`$ and $`y_0=x`$: both states are directly available through the two healthy outputs. The example evaluates the estimation and compensation architecture under this favorable measurement arrangement.

| Setting | Default |
| --- | --- |
| Design domain | $`[-2,2]\times[-2,2]`$ |
| State and input cost weights | `Qc = diag([4 1])`, `R = 0.5` |
| Initial nominal policy | `mu_0(x) = -[1.6 1.2] * x` |
| Observer adaptation and leakage | $`\Gamma=3`$, $`\delta=0.25`$ |
| Initial conditions | `x0 = [1.2; -0.7]`, `xhat0 = [0; 0]`, `fahat0 = 0` |
| Simulation interval | 0 to 25 s |
| Recorded output spacing | 0.01 s |
| Integration | `ode45`, maximum step 0.02 s |
| Relative / absolute tolerance | `1e-7` / `1e-9` |
| Input limit | Unbounded by default (`uMax = inf`) |
| ADP iteration limit | 20, with a minimum of 3 iterations |

## Requirements

- **MATLAB:** the project report specifies R2024a. Compatibility with earlier releases has not been established.
- **Optional LMI-design dependencies:** YALMIP and a compatible semidefinite programming solver. The report uses SDPT3. The code explicitly selects SDPT3 when `sqlp` is available; otherwise it uses YALMIP's automatic solver selection. These packages are external and are not included in the supplied source set.
- **Analytical observer fallback:** when YALMIP is unavailable or its solve fails, the supplied example can use an analytical design. It uses $`P=I`$, $`\rho=1`$, and $`L=A+2I`$, followed by the same numerical feasibility checks. This construction is specific to the demonstration arrangement with $`C_0=I_2`$; it requires redesign for another plant.

All 13 project MATLAB files are present. The workflow requires no external dataset, pretrained weights, or Simulink model. No calls to Control System Toolbox or Deep Learning Toolbox functions appear in the supplied implementation; the Lyapunov equation is solved directly using a Kronecker formulation.

## Quick start

Download or clone this repository. From the repository root in MATLAB, enter the source folder and run:

```matlab
cd('Matlab Code')
main_chapter4
```

All 13 `.m` files are in `Matlab Code/`. Keep that folder as the MATLAB Current Folder for the interactive examples below. The [project summary](Project_Summary.pdf) presents the methodology and numerical results.

The entry point checks structural assumptions, designs the observer, trains the nominal policy, runs all four fault scenarios, and generates comparison plots for scenarios 2 and 4. Inspect the structural-check table and observer diagnostics before interpreting the results. Structural-check failures produce a warning; they do not automatically stop the main workflow.

To run a scenario interactively and inspect the selected observer design:

```matlab
p = system_parameters();
checks = check_assumptions(p);
obs = observer_lmi_design(p);
adp = adp_training(p);

disp(obs.solverInfo)
disp(obs.usedFallback)

result = run_simulation(p, obs, adp, 4, 'proposed');
metrics = compute_metrics(result, p);
plot_chapter4_results(result, p);
```

### Fault scenarios

| ID | Scenario | Actuator fault | Sensor fault |
| --- | --- | --- | --- |
| 1 | Fault-free | Zero | Zero |
| 2 | Actuator fault | Smooth window with sinusoidal ripple | Zero |
| 3 | Sensor fault | Zero | Piecewise-constant bias |
| 4 | Simultaneous faults | Same as scenario 2 | Same as scenario 3 |

The default actuator profile is

```math
f_a(t)=\left[1+0.10\sin\left(0.8(t-5)\right)\right]\left[s(t-5)-s(t-17)\right]
```

```math
s(\tau)=\frac{1}{2}\left[1+\tanh(3\tau)\right]
```

The times 5 s and 17 s are transition centers; the smooth profile has small tails outside that interval. The sensor bias is zero before 8 s, 0.45 from 8 s to 16 s, and 0.20 thereafter through the end of the simulation.

### Comparison modes

| Mode | Nominal feedback | Actuator-fault compensation |
| --- | --- | --- |
| `proposed` | Learned policy evaluated at $`\hat{x}`$ | Estimated fault |
| `no_compensation` | Same learned policy evaluated at $`\hat{x}`$ | Disabled |
| `ideal` | Same learned policy evaluated at the true state $`x`$ | Exact fault |

The uncompensated baseline retains the adaptive observer. The ideal-information mode provides a reference with exact state and fault information; its nominal policy is still the learned ADP policy.

### Fault-amplitude sweep

Run the sweep separately from `main_chapter4`:

```matlab
T = plot_fault_amplitude_sweep;                  % Default amplitudes, scenario 2
T = plot_fault_amplitude_sweep(0:0.2:1.4, 4);    % Simultaneous faults
```

The default amplitudes are `[0 0.4 0.7 1.0 1.3]`. The observer and policy are designed once per sweep and reused across amplitudes. Each amplitude compares `proposed` and `no_compensation` and records state ISE, finite-horizon cost, percentage reductions, domain checks, and saturation flags.

**Profile difference:** the sweep sets the ripple amplitude to `(0.10/0.70) * amplitude`. At amplitude 1, its ripple is approximately 0.142857, while the main workflow uses 0.10. Consequently, the amplitude-1 sweep and the main actuator-fault scenario evaluate different fault profiles.

## Outputs

`main_chapter4` writes to `results/` beside the source files. In this repository, new runs therefore write to `Matlab Code/results/`; the top-level `results/` directory contains the uploaded result archives.

The generated outputs are:

- `chapter4_metrics.csv`: four rows of metrics for the `proposed` mode.
- `chapter4_results.mat`: parameters, structural checks, observer design, ADP policy and history, proposed trajectories, comparison trajectories, and the summary table.
- `adp_training_diagnostics.png`: critic weights, Bellman RMSE, minimum regression singular value, and policy acceptance coefficient.
- Four plots per scenario: states, faults, estimation errors, and control/output signals.
- `comparison_scenario_2.png` and `comparison_scenario_4.png`.

The main run generates 19 PNG files. Figures are exported at 300 dpi. Comparison trajectories are saved in the MAT file; the main CSV contains only the proposed-mode metrics.

The sweep writes scenario-specific PNG, CSV, and MAT files under `Matlab Code/results/amplitude_sweep/`. Repeated runs overwrite matching output filenames.

Metrics include state-estimation RMSE, actuator- and sensor-fault RMSE, state ISE, control energy, peak norms, settling time, and

```math
J_T=\int_0^T\left(x^TQ_cx+u^TRu\right)\,\mathrm{d}t
```

`J_T` uses the total applied control, including compensation. Fault RMSE uses samples where the true fault norm exceeds `1e-5`, or the full trajectory when no such samples exist. Settling time is an absolute simulation time after the scenario's designated last event, using a state-norm threshold of 0.05; it is not elapsed time since that event.

## Source files

| File | Role |
| --- | --- |
| `main_chapter4.m` | Complete simulation, comparison, and export workflow |
| `system_parameters.m` | Plant, design, fault, simulation, and output settings |
| `nonlinear_dynamics.m` | Known nonlinear plant term |
| `check_assumptions.m` | Structural rank, projection, matching, and sampled Rosenbrock checks |
| `observer_lmi_design.m` | Observer synthesis, analytical fallback, and feasibility checks |
| `adp_training.m` | Offline critic fitting and policy iteration |
| `policy_certification.m` | Finite-grid Lyapunov screening and policy blending |
| `fault_profiles.m` | Four predefined fault scenarios |
| `closed_loop_dynamics.m` | Plant, adaptive observer, control, and accumulated-cost dynamics |
| `run_simulation.m` | Numerical integration and trajectory reconstruction |
| `compute_metrics.m` | Quantitative performance measures |
| `plot_chapter4_results.m` | Scenario figures |
| `plot_fault_amplitude_sweep.m` | Separate fault-amplitude sensitivity study |

## Reproducibility and scope

The accompanying report develops a descriptor interpretation and conditional Lyapunov results for uniform ultimate boundedness. The executable simulation integrates ordinary differential equations for the plant and adaptive observer; sensor-fault reconstruction is algebraic.

The following qualifications apply:

- **Numerical screening:** policy acceptance is checked at finitely many grid points. It does not establish a uniform Lyapunov inequality or an invariant set throughout the continuous design domain. Domain checks also apply only at recorded trajectory samples.
- **Theoretical bound:** `p.theory.rc` defaults to `NaN`. The implementation does not compute the constants, policy-error bound, or invariant set needed to instantiate the report's composite stability theorem. The plotted aggregate norm is a simulation diagnostic.
- **Optimality:** weight convergence and a small Bellman residual do not establish a bound on distance to the optimal policy or guarantee monotonic improvement of the implemented compensated controller's cost.
- **Measurement assumptions:** the fault distribution matrices and plant model are known. The example uses exact, noise-free measurements. `p.sim.noiseStd` is declared but is not used to inject noise; changing it alone has no effect.
- **Estimation error:** leakage can produce a bounded actuator-fault estimation bias. Sensor-fault reconstruction inherits state-estimation error, including during overlapping actuator faults.
- **Input limits:** finite `p.sim.uMax` enables input clipping, but the documented stability analysis assumes unsaturated control.
- **Generalization:** the critic and grid routines explicitly support two states and one control input. The supplied fault generator populates only the first actuator and sensor fault channels. Applying the method to another system requires updating these routines, the nonlinearity, projection, structural assumptions, and observer design.

### Reported results and saved artifacts

The stored parameter snapshots distinguish the main experiment from the other archives:

| Configuration | Actuator amplitude | Ripple amplitude | Proposed state ISE | Proposed cost `J_T` |
| --- | --- | --- | --- | --- |
| `results/1/`, matching the main comparison in the report | 1.0 | 0.10 | 2.3434 | 9.9271 |
| Top-level `results/` archive | 1.3 | 0.10 | 2.9032 | 13.4041 |
| Saved amplitude sweep at amplitude 1.0 | 1.0 | 0.142857 | 2.3703 | 10.0633 |

The top-level archive was generated with actuator amplitude 1.3; the current source defaults to 1.0. Its values therefore should not be labeled as an amplitude-1 experiment. The sweep/main ripple difference is described above. The report's numerical sweep table also differs slightly from the supplied sweep CSV; use the specific archive and its parameters when quoting results.

Observer gains depend on whether the solver or analytical fallback is used. Record the actual parameters, solver information, and generated outputs for each run before reporting improvements. Compensation can improve regulation while increasing control effort, and it can increase total cost in the fault-free case.

No required project source file is missing. External solver packages, a fully specified solver environment, and an implemented numerical calculation of the theoretical ultimate bound are not supplied.
