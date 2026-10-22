# Third-party notices

## KF-GINS backbone

The inertial mechanization, state representation and error-state filtering backbone are derived and refactored from KF-GINS and the KF-GINS graduation-design code. The source reference used for the port is [KF-GINS-graduation-design](https://github.com/kaiwen123-yang/KF-GINS-graduation-design), revision `5a4471efd4fcfcdc31e258a677af354c652ff16f`. Its GNU GPL v3 license is preserved as the root `LICENSE`. The upstream KF-GINS project is maintained by the [i2Nav research group](https://github.com/i2Nav-WHU/KF-GINS).

The files in this repository are modified implementations. LegSA-GINS adds short-baseline heading selection, Doppler and robot-motion observations, and source weighting. The inherited inertial backbone is not presented as a new contribution.

## RTKLIB

The raw GNSS bridge and Doppler preparation tools use [RTKLIB](https://github.com/tomojitakasu/RTKLIB), revision `180043ee24b6d2b168f98b64be15f69d50046b1a`. The RTKLIB source itself is obtained separately. Its copyright and redistribution terms are reproduced in `LICENSES/RTKLIB.txt`. The bridge, velocity-export helper and observation adapters are project integration code.

## Numerical dependencies

The main C++ estimator has no third-party matrix-library dependency. The contact-inertial comparison uses Eigen. Python tools use NumPy, SciPy, pandas, PyYAML and Matplotlib. OiSAM additionally uses GTSAM 4.2 and pyceres 2.6/Ceres. These libraries are installed separately and retain their respective licenses.

## Comparison implementations

C-LAMBDA, C-WLS, the position-aided invariant EKF, the contact-inertial comparison, OiSAM, TC-FGO and GNC-FGO are the project implementations/adaptations used in the manuscript. They are not distributed as the original authors' complete software. Method-specific observation sets, parameter choices, gap handling and output-point conventions are described in `docs/comparisons.md` and the associated configuration files.
