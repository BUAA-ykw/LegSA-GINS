# LegSA-GINS

Companion source code for **LegSA-GINS: Motion-constrained short-baseline GNSS/INS integration for compact legged robots**.

LegSA-GINS combines inertial propagation, receiver position and velocity, short-baseline GNSS heading, Doppler-derived velocity, and robot inclination and horizontal velocity in one error-state filter. Source-specific covariance inflation adjusts the influence of the observations. This repository contains the implementation and experiment definitions used in the manuscript.

## Build and run

The main estimator requires a C++17 compiler and CMake 3.16 or newer. The Python tools require Python 3.10 or newer. The release was checked on Ubuntu 22.04 with GCC 11.4 and Python 3.10.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel

python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .

python -m experiments.run \
  --sequence BY2 --profile full \
  --data-root /path/to/prepared_data \
  --output outputs/BY2/full
```

Raw recordings, reference trajectories and previously generated results are **not included in this repository**. Reproduction requires access to the corresponding inputs described in [Data formats](docs/data_format.md). Building the estimator does not require a dataset.

For a fully specified configuration, the executable can also be used directly:

```bash
build/legsa_gins --config configuration.yaml --output-dir outputs/example
```

## Repository layout

| Directory | Purpose |
|---|---|
| `include/legsa_gins`, `src/core`, `src/main.cpp` | Navigation estimator, observation models, readers and executable |
| `preprocessing` | Robot/receiver decoding, frame conversions, Doppler preparation and heading construction |
| `configs` | Fixed sequence parameters, method profiles and controlled experiment definitions |
| `experiments` | Run entry point, deterministic perturbations and interruption definitions |
| `baselines` | Heading, invariant EKF and factor-graph comparison implementations |
| `evaluation` | Manuscript evaluator, midpoint transformation and error statistics |
| `plotting` | Rendering routines for the manuscript's numerical figures |
| `docs` | Input formats, reproduction instructions and comparison interfaces |

## Manuscript configurations

| Profile | Observations enabled after common initialization |
|---|---|
| `no_online_heading` | GNSS position and receiver velocity |
| `position_heading` | GNSS position and basic dual-antenna heading |
| `position_velocity_heading` | GNSS position, receiver velocity and qualified heading |
| `full` | Full LegSA-GINS observation set and source weighting |
| `without_doppler` | Full method with Doppler velocity disabled |
| `without_weighting` | Full method with source weighting disabled |
| `without_inclination` | Full method with robot roll/pitch disabled |
| `without_horizontal_velocity` | Full method with robot horizontal velocity disabled |

All profiles retain the common dual-antenna initialization. The `no_online_heading` profile disables subsequent heading updates. The fixed windows are BY2: 66–340 s, BY2H: 413–683 s, and BY2O: 3186–3563 s.

The estimator consumes a prepared horizontal velocity. Its heading support and the valid GNSS event that schedules auxiliary updates are part of the implemented method. Complete GNSS loss and translation-only loss with heading preserved are distinct experiments.

## Evaluation

```bash
python -m evaluation.run \
  --sequence BY2 --run outputs/BY2/full \
  --reference /path/to/reference.csv \
  --output outputs/BY2/evaluation
```

Navigation states are produced at the IMU. Position evaluation uses the declared antenna midpoint; the supplied transform changes only the three position columns. Reference trajectories are used only by the evaluation tools. The commercial fused reference is not an independently established ground truth. Reported covariance diagonals are not transported to the midpoint.

## Comparisons and figures

Install the optional factor-graph dependencies with `python -m pip install -e '.[comparisons]'`. The comparison methods have different input sets and output states. Their interfaces, fixed parameters and build steps are described in [Comparisons](docs/comparisons.md).

The [Reproduction guide](docs/reproduction.md) maps the manuscript experiments and figures to the supplied code. Numerical figure inputs must be supplied separately. Platform photographs and illustrative artwork are not included in this code repository.

## Implementation validation

The release estimator was compared with the manuscript implementation on all eight BY2 profiles and on the full BY2H and BY2O runs. All ten comparisons produced byte-identical navigation, covariance-standard-deviation and IMU-error outputs. The main estimator, contact-inertial executable and RTKLIB bridge were built separately. The six numerical rendering entry points were exercised using the retained manuscript figure inputs.

These checks validate the source extraction and execution interfaces; they do not constitute a new execution of the complete 541-case matrix or all external comparisons.

## License and attribution

The source is provided under the GNU General Public License, version 3. See [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for the KF-GINS backbone, external libraries and comparison implementations.
