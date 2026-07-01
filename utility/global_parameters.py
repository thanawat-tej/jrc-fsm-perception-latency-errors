#  Copyright (c) 2021 European Union
#  *
#  Licensed under the EUPL, Version 1.2 or – as soon they will be approved by the
#  European Commission – subsequent versions of the EUPL (the "Licence");
#  You may not use this work except in compliance with the Licence.
#  You may obtain a copy of the Licence at:
#  *
#  https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12
#  *
#  Unless required by applicable law or agreed to in writing,
#  software distributed under the Licence is distributed on an "AS IS" basis,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the Licence for the specific language governing permissions and limitations
#  under the Licence.
#
#

### global parameters shared among models and simulations ###

# vehicle dimensions
length = 4.3
width = 1.9

# simulation parameters
iterations = 350
freq = 10
wandering_zone = -0.375
g = 9.81

# ---------------------------------------------------------------------------
# Perception-noise baseline (Gaussian)
# ---------------------------------------------------------------------------
# The ego's *perceived* state of the other vehicle (the cut-in / lead / cut-out
# actor) is perturbed by independent N(0, sigma^2) noise at every timestep.
# The TRUE vehicle state -- used for crash detection and for the other
# vehicle's own motion -- is never modified, so this isolates perception error
# from the underlying dynamics.
#
# Set sigma > 0 to enable. Both default to 0.0, which reproduces the original
# perfect-perception behaviour exactly (the noise layer is skipped entirely).
perception_noise_sigma_pos = 0.0     # std-dev of position noise [m]   (long & lat)
perception_noise_sigma_speed = 0.0   # std-dev of speed noise [m/s]    (long & lat)
perception_noise_seed = 12345        # base RNG seed for reproducibility

# Sensing latency: the ego perceives the other vehicle's state from t - lambda.
# The delay is converted to whole steps (d = round/Poisson of mean_s * freq).
# mean_s = 0.0 disables latency. Mode selects how the per-step delay is drawn:
#   'constant'      -> fixed delay every step
#   'fixed_poisson' -> one Poisson draw per scenario, held constant
#   'poisson'       -> fresh Poisson draw every step (jittery)
perception_latency_mean_s = 0.0      # mean perception latency lambda [s]
perception_latency_mode = 'constant'
