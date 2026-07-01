'''Functions relevant to vehicle movement'''

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

from . import vehicle as vi
from . import global_parameters as gp
import numpy as np
import hashlib
import warnings


# ===========================================================================
# Perception-error layer: Gaussian measurement noise + sensing latency
# ===========================================================================
# The safety models read the other vehicle's state directly from its profile
# arrays (e.g. cutting_in_veh.pos_profile_long[i]). To model imperfect
# perception we hand the model a lightweight READ-ONLY shadow of that vehicle:
#   * LATENCY -- the perceived state at step i is the TRUE state from an earlier
#     step (i - d), where the delay d (in steps) may be constant or random
#     (Poisson). This models a sensing/processing pipeline that delivers data
#     measured at t - lambda.
#   * NOISE   -- that delayed state is then perturbed by independent N(0,sigma^2)
#     Gaussian measurement noise on position (long+lat) and speed (long+lat).
# Every other index and every geometric/dynamic attribute defers to the true
# vehicle, which is left untouched, so crash detection and the other vehicle's
# own motion still use ground truth.

def _normalize_seed(seed):
    """Coerce any seed (int, negative int, float, or tuple of scenario params)
    to a non-negative 32-bit int that numpy's default_rng accepts. Uses a
    stable hash so the mapping is deterministic across processes (unlike the
    salted built-in hash()), preserving reproducibility and cross-model
    identity of the per-cell seed tuples."""
    if seed is None:
        return None
    if isinstance(seed, (int, np.integer)) and seed >= 0:
        return int(seed)
    return int(hashlib.sha256(repr(seed).encode()).hexdigest(), 16) % (2 ** 32)

class _NoisyArray(object):
    """Array-like proxy that returns a perturbed value at index `i` and the
    true value everywhere else. Only `[i]`-style scalar reads are used by the
    models for the perceived vehicle; whole-array access falls back to truth."""
    __slots__ = ('_a', '_i', '_val')

    def __init__(self, true_array, i, noisy_value):
        self._a = true_array
        self._i = i
        self._val = noisy_value

    def __getitem__(self, k):
        if k == self._i:
            return self._val
        return self._a[k]

    def __len__(self):
        return len(self._a)

    def __array__(self, dtype=None):
        return np.asarray(self._a, dtype=dtype)


class _NoisyView(object):
    """Read-only shadow of a vehicle whose perceived state at step `i` is the
    TRUE state from source step `src` (= i - latency) plus Gaussian noise.
    Position (long+lat) and speed (long+lat) are perturbed; geometry
    (width/length) and dynamic limits defer to the true vehicle."""
    __slots__ = ('_v', 'pos_profile_long', 'pos_profile_lat',
                 'speed_profile_long', 'speed_profile_lat')

    def __init__(self, veh, i, src, rng, sigma_pos, sigma_speed):
        self._v = veh
        self.pos_profile_long = _NoisyArray(
            veh.pos_profile_long, i, veh.pos_profile_long[src] + rng.normal(0.0, sigma_pos))
        self.pos_profile_lat = _NoisyArray(
            veh.pos_profile_lat, i, veh.pos_profile_lat[src] + rng.normal(0.0, sigma_pos))
        self.speed_profile_long = _NoisyArray(
            veh.speed_profile_long, i, veh.speed_profile_long[src] + rng.normal(0.0, sigma_speed))
        self.speed_profile_lat = _NoisyArray(
            veh.speed_profile_lat, i, veh.speed_profile_lat[src] + rng.normal(0.0, sigma_speed))

    def __getattr__(self, name):
        # width, length, max_d, max_a_lat, crash, ... -> true vehicle
        return getattr(self._v, name)


class PerceptionNoise(object):
    """Perception-error model applied to the ego's view of the other vehicle.

    Two effects, either of which can be disabled (set to 0):
      * Gaussian measurement noise: independent N(0, sigma^2) per timestep on
        perceived position (long+lat) and speed (long+lat).
      * Sensing latency: the perceived state at step i is the TRUE state from
        step i - d, where d is the delay in steps. latency_mode selects how d
        is drawn each step:
          'constant'      -> d = round(latency_mean_s * freq)        (fixed)
          'fixed_poisson' -> d ~ Poisson(latency_mean_s * freq), drawn ONCE
                             per scenario and then held constant
          'poisson'       -> d ~ Poisson(latency_mean_s * freq), redrawn every
                             step (jittery perceived timestamp)
    """

    def __init__(self, sigma_pos=0.0, sigma_speed=0.0, seed=None,
                 latency_mean_s=0.0, latency_mode='constant'):
        self.sigma_pos = float(sigma_pos)
        self.sigma_speed = float(sigma_speed)
        self.latency_mean_s = float(latency_mean_s)
        self.latency_mode = str(latency_mode)
        self.active = (self.sigma_pos > 0.0 or self.sigma_speed > 0.0
                       or self.latency_mean_s > 0.0)
        self.rng = np.random.default_rng(_normalize_seed(seed))
        self._fixed_delay = None   # cache for 'fixed_poisson'
        self.trace = None          # set to [] via record() to log every view

    def record(self):
        """Enable full per-step logging of perceived-vs-true state. Intended for
        a single scenario (one_case); the comparison sweep should NOT record."""
        self.trace = []
        return self

    def _delay_steps(self, freq):
        if self.latency_mean_s <= 0.0:
            return 0
        mean_steps = self.latency_mean_s * freq
        if self.latency_mode == 'poisson':
            return int(self.rng.poisson(mean_steps))
        if self.latency_mode == 'fixed_poisson':
            if self._fixed_delay is None:
                self._fixed_delay = int(self.rng.poisson(mean_steps))
            return self._fixed_delay
        # 'constant' (default)
        return int(round(mean_steps))

    def view(self, veh, i, freq):
        if not self.active or veh is None:
            return veh
        src = i - self._delay_steps(freq)
        if src < 0:
            src = 0   # cannot perceive before the trajectory starts
        v = _NoisyView(veh, i, src, self.rng, self.sigma_pos, self.sigma_speed)
        if self.trace is not None:
            # Read the perceived values straight back from the view (the noisy
            # values already drawn) so we record exactly what the model sees.
            self.trace.append({
                'i': i, 'delay_steps': i - src,
                'perc_long': float(v.pos_profile_long[i]),
                'true_long': float(veh.pos_profile_long[i]),
                'perc_lat': float(v.pos_profile_lat[i]),
                'true_lat': float(veh.pos_profile_lat[i]),
                'perc_speed_long': float(v.speed_profile_long[i]),
                'true_speed_long': float(veh.speed_profile_long[i]),
                'perc_speed_lat': float(v.speed_profile_lat[i]),
                'true_speed_lat': float(veh.speed_profile_lat[i]),
            })
        return v


def noise_suffix():
    """Path suffix encoding the active perception-error level, or '' when both
    noise and latency are disabled. Lets perturbed runs write to their own
    folders instead of overwriting the clean (perfect-perception) results.
    E.g. '_noise_p0.5_s1', '_noise_lat0.3poisson', '_noise_p0.5_s1_lat0.3constant'."""
    sp = gp.perception_noise_sigma_pos
    ss = gp.perception_noise_sigma_speed
    lat = gp.perception_latency_mean_s
    parts = []
    if sp > 0.0 or ss > 0.0:
        parts.append('p%g_s%g' % (sp, ss))
    if lat > 0.0:
        parts.append('lat%g%s' % (lat, gp.perception_latency_mode))
    if not parts:
        return ''
    return '_noise_' + '_'.join(parts)


def make_perception_noise(seed=None):
    """Build a PerceptionNoise from the global_parameters knobs.

    `seed` overrides the base seed (gp.perception_noise_seed). Pass a tuple of
    scenario parameters to make each scenario cell reproducible AND identical
    across models, e.g. make_perception_noise(seed=(gp.perception_noise_seed,
    ego_speed, cut_in_speed)). Returns an inert (active=False) object when noise
    and latency are both off, so callers can always pass the result to control()."""
    base = gp.perception_noise_seed if seed is None else seed
    return PerceptionNoise(gp.perception_noise_sigma_pos,
                           gp.perception_noise_sigma_speed,
                           base,
                           latency_mean_s=gp.perception_latency_mean_s,
                           latency_mode=gp.perception_latency_mode)


def noise_active():
    """True when any perception error (Gaussian noise or latency) is enabled."""
    return (gp.perception_noise_sigma_pos > 0.0 or
            gp.perception_noise_sigma_speed > 0.0 or
            gp.perception_latency_mean_s > 0.0)


def write_perception_trace(noise, out_path, freq):
    """Write the full per-step perceived-vs-true trajectory recorded by a
    PerceptionNoise (see record()). Returns False if there is nothing to write
    (recording was off, or noise/latency disabled). One row per control step:
    time, delay, and perceived vs true long/lat position and speed."""
    import csv as _csv
    import os as _os
    if noise is None or not getattr(noise, 'trace', None):
        return False
    cols = ['t', 'i', 'delay_steps',
            'perc_long', 'true_long', 'long_err',
            'perc_lat', 'true_lat', 'lat_err',
            'perc_speed_long', 'true_speed_long', 'speed_long_err',
            'perc_speed_lat', 'true_speed_lat', 'speed_lat_err']
    parent = _os.path.dirname(out_path)
    if parent:
        _os.makedirs(parent, exist_ok=True)
    with open(out_path, 'w', newline='') as f:
        w = _csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in noise.trace:
            w.writerow({
                't': round(r['i'] / freq, 3), 'i': r['i'],
                'delay_steps': r['delay_steps'],
                'perc_long': round(r['perc_long'], 4),
                'true_long': round(r['true_long'], 4),
                'long_err': round(r['perc_long'] - r['true_long'], 4),
                'perc_lat': round(r['perc_lat'], 4),
                'true_lat': round(r['true_lat'], 4),
                'lat_err': round(r['perc_lat'] - r['true_lat'], 4),
                'perc_speed_long': round(r['perc_speed_long'], 4),
                'true_speed_long': round(r['true_speed_long'], 4),
                'speed_long_err': round(r['perc_speed_long'] - r['true_speed_long'], 4),
                'perc_speed_lat': round(r['perc_speed_lat'], 4),
                'true_speed_lat': round(r['true_speed_lat'], 4),
                'speed_lat_err': round(r['perc_speed_lat'] - r['true_speed_lat'], 4),
            })
    return True


def create_profile_cutting_in(init_pos, init_long_speed, lateral_speed, iterations, freq):

    speed_profile_long = np.array([init_long_speed] * iterations)
    if lateral_speed != 0:
        cut_in_duration = int(np.abs(freq * init_pos[1]) // np.abs(lateral_speed) + 1)
        speed_profile_lat = np.array(
            [lateral_speed] * cut_in_duration + [0] * (iterations - cut_in_duration))
    else:
        speed_profile_lat = np.array([0.] * iterations)
    speed_profile_lat = speed_profile_lat[:iterations]
    pos_profile_long = speed_profile_long.cumsum() / freq + init_pos[0]
    pos_profile_lat = speed_profile_lat.cumsum() / freq + init_pos[1]

    veh = vi.vehicle()
    veh.speed_profile_lat = speed_profile_lat
    veh.speed_profile_long = speed_profile_long
    veh.pos_profile_lat = pos_profile_lat
    veh.pos_profile_long = pos_profile_long

    return veh


def create_profile_decel(init_pos, deceleration, init_long_speed, iterations, freq):  ###NO JERK no nothing

    # speed_profile_long = np.array([init_long_speed] * iterations)
    speed_profile_long = np.array(init_long_speed - np.arange(0, iterations, 1) * deceleration / freq)
    speed_profile_long[speed_profile_long < 0] = 0
    speed_profile_lat = np.array([0] * iterations)

    pos_profile_long = speed_profile_long.cumsum() / freq + init_pos[0]
    pos_profile_lat = speed_profile_lat.cumsum() / freq + init_pos[1]

    veh = vi.vehicle()
    veh.speed_profile_lat = speed_profile_lat
    veh.speed_profile_long = speed_profile_long
    veh.pos_profile_lat = pos_profile_lat
    veh.pos_profile_long = pos_profile_long

    return veh


def control(ego_veh, cutting_in_veh, freq, model_check, model_react, i,
            perception=None):
    speed_log = ego_veh.speed_profile_long[i]
    speed_lat = ego_veh.speed_profile_lat[i]

    # The model decides on the (possibly noisy) PERCEIVED other vehicle; the
    # true cutting_in_veh is still used below for crash detection.
    perceived_veh = cutting_in_veh
    if perception is not None:
        perceived_veh = perception.view(cutting_in_veh, i, freq)

    safe = model_check(ego_veh, perceived_veh, speed_log, speed_lat, freq, i)

    # Perception snapshot at CLOSEST APPROACH: on every step we keep the
    # perceived-vs-true sample from the step with the smallest TRUE longitudinal
    # gap to the currently-tracked actor. This fills EVERY cell (not only the
    # ones where the ego braked). With noise off, perceived == true.
    # Gaps are edge-to-edge CLEARANCES, exactly as the models and the crash test
    # measure them (a crash needs BOTH < 0 at one step):
    #   long_gap = bumper-to-bumper longitudinal clearance [m]
    #   lat_gap  = side-to-side lateral clearance [m]   (<=0 => in-lane)
    half_len = ego_veh.length / 2 + cutting_in_veh.length / 2
    half_wid = ego_veh.width / 2 + cutting_in_veh.width / 2
    ego_long = ego_veh.pos_profile_long[i]
    ego_lat = ego_veh.pos_profile_lat[i]
    true_gap = abs(cutting_in_veh.pos_profile_long[i] - ego_long) - half_len
    if true_gap < getattr(ego_veh, 'snap_min_gap', np.inf):
        ego_veh.snap_min_gap = true_gap
        ego_veh.snap_true_gap = true_gap
        ego_veh.snap_perc_gap = abs(perceived_veh.pos_profile_long[i] - ego_long) - half_len
        ego_veh.snap_true_lat = abs(cutting_in_veh.pos_profile_lat[i] - ego_lat) - half_wid
        ego_veh.snap_perc_lat = abs(perceived_veh.pos_profile_lat[i] - ego_lat) - half_wid
        ego_veh.snap_true_speed = cutting_in_veh.speed_profile_long[i]
        ego_veh.snap_perc_speed = perceived_veh.speed_profile_long[i]
        # Ego's own speed at this frame (true), and perceived/true lateral
        # closing speed of the actor (used to map u_t and v_y_rel downstream).
        ego_veh.snap_ego_speed = ego_veh.speed_profile_long[i]
        ego_veh.snap_true_lat_speed = cutting_in_veh.speed_profile_lat[i]
        ego_veh.snap_perc_lat_speed = perceived_veh.speed_profile_lat[i]

    # LANE-ENTRY snapshot: the FIRST frame the actor enters the ego's lateral
    # footprint (lateral clearance crosses <= 0). For a cut-in this is the
    # dangerous instant -- the lateral closing speed is still active and the
    # actor is at the lane edge -- which the closest-approach (min-gap) snapshot
    # misses because by min-gap the merge is over and the ego has already
    # braked. Every collision cell has such a frame (a crash needs lateral
    # overlap), so this gives the cut-in classifier a frame it can flag.
    if not getattr(ego_veh, 'snap_entry_done', False):
        lat_clear = abs(cutting_in_veh.pos_profile_lat[i] - ego_lat) - half_wid
        if lat_clear <= 0:
            ego_veh.snap_entry_done = True
            ego_veh.snap_entry_true_gap = abs(cutting_in_veh.pos_profile_long[i] - ego_long) - half_len
            ego_veh.snap_entry_perc_gap = abs(perceived_veh.pos_profile_long[i] - ego_long) - half_len
            ego_veh.snap_entry_true_lat = lat_clear
            ego_veh.snap_entry_perc_lat = abs(perceived_veh.pos_profile_lat[i] - ego_lat) - half_wid
            ego_veh.snap_entry_true_speed = cutting_in_veh.speed_profile_long[i]
            ego_veh.snap_entry_perc_speed = perceived_veh.speed_profile_long[i]
            ego_veh.snap_entry_ego_speed = ego_veh.speed_profile_long[i]
            ego_veh.snap_entry_true_lat_speed = cutting_in_veh.speed_profile_lat[i]
            ego_veh.snap_entry_perc_lat_speed = perceived_veh.speed_profile_lat[i]

    if safe and ego_veh.safe:
        new_speed_long = min(ego_veh.speed_profile_long[i + 1], speed_log + ego_veh.max_a_CF / freq)
        new_speed_long = max(new_speed_long, speed_log - ego_veh.max_a_CF / freq)

        new_speed_lat = min(ego_veh.speed_profile_lat[i + 1], speed_lat + ego_veh.max_a_lat / freq)
        new_speed_lat = max(new_speed_lat, speed_lat - ego_veh.max_a_lat / freq)

    elif safe and not ego_veh.safe:
        new_speed_long, new_speed_lat = speed_log, speed_lat

    else:
        new_speed_long, new_speed_lat = model_react(ego_veh, speed_log, freq)
        ego_veh.safe = False

    ego_veh.speed_profile_long[i + 1] = new_speed_long
    ego_veh.speed_profile_lat[i + 1] = new_speed_lat

    ego_veh.pos_profile_long[i + 1] = ego_veh.pos_profile_long[i] + new_speed_long / freq
    ego_veh.pos_profile_lat[i + 1] = ego_veh.pos_profile_lat[i] + new_speed_lat / freq

    if abs(ego_veh.pos_profile_lat[i] - cutting_in_veh.pos_profile_lat[i]) +\
            - ego_veh.width / 2 - cutting_in_veh.width / 2 < 0 and \
            abs(ego_veh.pos_profile_long[i] - cutting_in_veh.pos_profile_long[i]) - ego_veh.length / 2 + \
            - cutting_in_veh.length / 2 < 0:

        if not ego_veh.crash:
            if ego_veh.pos_profile_long[i] > cutting_in_veh.pos_profile_long[i]:
                ego_veh.crash_type = 1
            else:
                ego_veh.crash_type = 2
        ego_veh.crash = True
