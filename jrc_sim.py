"""
jrc_sim.py -- vendored JRC-FSM closed-loop cut-in simulation (importable module).

Derived from ec-jrc/JRC-FSM, Copyright (c) 2021 European Union,
licensed under the EUPL v1.2 (https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12).
This file is a derivative work under the EUPL v1.2. numba decorators removed,
logic unchanged except: (a) the cut-in vehicle's initial lateral offset is a
parameter instead of the hardcoded 1.6+W, (b) tiny epsilons guard zero-division
in the TTC expressions. These generalizations let the sim classify arbitrary
corrected.csv rows (whose lateral gap varies) rather than only the fixed grid.

Exact closed-loop realizations of the R157 driver models for the CUT-IN family:
  CC_human_driver (the JAMA-lineage competent-and-careful driver), RSS, FSM, Reg157.
The crash frontier of CC_human_driver is the computable JAMA-lineage boundary.
"""
import numpy as np

J_LEN, J_WID, J_ITER, J_FREQ = 4.3, 1.9, 350, 10
_EPS = 1e-6


class JVeh:
    def __init__(self):
        self.speed_profile_long = []; self.speed_profile_lat = []
        self.pos_profile_long = []; self.pos_profile_lat = []
        self.width = J_WID; self.length = J_LEN
        self.max_a = 3; self.max_a_CF = 1; self.max_d = 6; self.max_a_lat = 1
        self.safe = True; self.crash = False; self.deceleration = 0
        self.CC_rt_counter = 0.75; self.CC_min_jerk = 12.65
        self.CC_max_deceleration = 0.774 * 9.81; self.CC_release_deceleration = 0.4; self.CC_critical_ttc = 2
        self.Reg157_rt_counter = 0.35; self.Reg157_max_deceleration = 6; self.Reg157_lat_safe_dist = 0.5
        self.RSS_rt_counter = 0.75; self.RSS_min_jerk = 12.65
        self.RSS_max_deceleration = 0.774 * 9.81; self.RSS_mu = 0.3
        self.FSM_rt = 0.75; self.FSM_rt_counter = 0.75; self.FSM_br_min = 4; self.FSM_br_max = 6
        self.FSM_bl = 7; self.FSM_ar = 2; self.FSM_margin_dist = 2; self.FSM_margin_safe_dist = 2
        self.FSM_max_deceleration = 0.774 * 9.81
        self.cfs = 0; self.pfs = 0


def j_profile_cutting_in(init_pos, v_long, v_lat, it, freq):
    sl = np.array([float(v_long)] * it)
    if v_lat != 0:
        dur = int(np.abs(freq * init_pos[1]) // np.abs(v_lat) + 1)
        sla = np.array([float(v_lat)] * min(dur, it) + [0.0] * max(0, it - dur))
    else:
        sla = np.array([0.0] * it)
    v = JVeh()
    v.speed_profile_long = sl; v.speed_profile_lat = sla[:it]
    v.pos_profile_long = sl.cumsum() / freq + init_pos[0]
    v.pos_profile_lat = v.speed_profile_lat.cumsum() / freq + init_pos[1]
    return v


def j_control(ego, cut, freq, check, react, i):
    slog = ego.speed_profile_long[i]; slat = ego.speed_profile_lat[i]
    safe = check(ego, cut, slog, slat, freq, i)
    if safe and ego.safe:
        nl = min(ego.speed_profile_long[i + 1], slog + ego.max_a_CF / freq); nl = max(nl, slog - ego.max_a_CF / freq)
        na = min(ego.speed_profile_lat[i + 1], slat + ego.max_a_lat / freq); na = max(na, slat - ego.max_a_lat / freq)
    elif safe and not ego.safe:
        nl, na = slog, slat
    else:
        nl, na = react(ego, slog, freq); ego.safe = False
    ego.speed_profile_long[i + 1] = nl; ego.speed_profile_lat[i + 1] = na
    ego.pos_profile_long[i + 1] = ego.pos_profile_long[i] + nl / freq
    ego.pos_profile_lat[i + 1] = ego.pos_profile_lat[i] + na / freq
    if (abs(ego.pos_profile_lat[i] - cut.pos_profile_lat[i]) - ego.width / 2 - cut.width / 2 < 0 and
            abs(ego.pos_profile_long[i] - cut.pos_profile_long[i]) - ego.length / 2 - cut.length / 2 < 0):
        ego.crash = True


def _PFS(dist, ur, ul, rt, br_min, br_max, bl, md, msd):
    dist = dist - md
    dsafe = ur * rt + ur ** 2 / (2 * br_min) - ul ** 2 / (2 * bl) + msd
    if dist > dsafe: return 0
    dunsafe = ur * rt + ur ** 2 / (2 * br_max) - ul ** 2 / (2 * bl)
    if dist < dunsafe: return 1
    return (dist - dsafe) / (dunsafe - dsafe)


def _CFS(dist, ur, ul, rt, br_min, br_max, bl, ar):
    arF = max(ar, -br_min); u_new = ur + rt * arF
    if ur <= ul: return 0
    if u_new < ul:
        dsafe = (ur - ul) ** 2 / max(abs(ar * 2), _EPS)
        return 1 if dist < dsafe else 0
    dsafe = (ur + arF * rt / 2 - ul) * rt + (ur + arF * rt - ul) ** 2 / (br_min * 2)
    if dist > dsafe: return 0
    dunsafe = (ur + arF * rt / 2 - ul) * rt + (ur + arF * rt - ul) ** 2 / (br_max * 2)
    if dist < dunsafe: return 1
    return (dist - dsafe) / (dunsafe - dsafe)


def CC_check(ego, cut, slog, slat, freq, i):
    if ego.pos_profile_long[i] > cut.pos_profile_long[i]: return True
    if abs(ego.pos_profile_lat[i] - cut.pos_profile_lat[i]) - ego.width / 2 - cut.width / 2 > 0: return True
    rel = ego.speed_profile_long[i] - cut.speed_profile_long[i]
    if abs(rel) < _EPS: return True
    if abs(abs(cut.pos_profile_long[i] - ego.pos_profile_long[i] - ego.length / 2 - cut.length / 2) / rel) > ego.CC_critical_ttc:
        return True
    return False


def CC_react(ego, slog, freq):
    if ego.CC_rt_counter > 0:
        ego.CC_rt_counter -= 1 / freq; ego.deceleration = ego.CC_release_deceleration
        return slog - ego.deceleration / freq, 0
    ego.deceleration = min(ego.deceleration + ego.CC_min_jerk / freq, ego.CC_max_deceleration)
    return max(slog - ego.deceleration / freq, 0), 0


def RSS_check(ego, cut, slog, slat, freq, i):
    if ego.pos_profile_long[i] > cut.pos_profile_long[i]: return True
    d = (slog * 0.75 + ego.max_a * 0.75 ** 2 / 2 +
         (slog + 0.75 * ego.max_a) ** 2 / (2 * ego.max_d) -
         cut.speed_profile_long[i] ** 2 / (2 * cut.max_d))
    if abs(ego.pos_profile_long[i] - cut.pos_profile_long[i]) - ego.length / 2 - cut.length / 2 < d:
        cl = abs(cut.speed_profile_lat[i])
        dl = (ego.RSS_mu + abs((2 * cl + cut.max_a_lat * 0.75) * 0.75 / 2) +
              (cl + cut.max_a_lat * 0.75) ** 2 / (2 * cut.max_a_lat))
        if abs(ego.pos_profile_lat[i] - cut.pos_profile_lat[i]) - ego.width / 2 - cut.width / 2 < dl:
            return False
    return True


def RSS_react(ego, slog, freq):
    if ego.RSS_rt_counter > 0:
        ego.RSS_rt_counter -= 1 / freq; return slog, 0
    ego.deceleration = min(ego.deceleration + ego.RSS_min_jerk / freq, ego.RSS_max_deceleration)
    return max(slog - ego.deceleration / freq, 0), 0


def Reg_check(ego, cut, slog, slat, freq, i):
    if ego.pos_profile_long[i] > cut.pos_profile_long[i]: return True
    if abs(ego.pos_profile_lat[i] - cut.pos_profile_lat[i]) - ego.width / 2 - cut.width / 2 > ego.Reg157_lat_safe_dist:
        return True
    rel = ego.speed_profile_long[i] - cut.speed_profile_long[i]
    if abs(rel) < _EPS: return True
    if abs((abs(ego.pos_profile_long[i] - cut.pos_profile_long[i]) - ego.length / 2 - cut.length / 2) / rel) > \
            rel / (2 * ego.Reg157_max_deceleration) + 0.35 + 0.1:
        return True
    return False


def Reg_react(ego, slog, freq):
    if ego.Reg157_rt_counter > 0:
        ego.Reg157_rt_counter -= 1 / freq; return slog, 0
    return max(slog - ego.Reg157_max_deceleration / freq, 0), 0


def FSM_check(ego, cut, slog, slat, freq, i):
    if ego.pos_profile_long[i] > cut.pos_profile_long[i]: return True
    if abs(ego.pos_profile_lat[i] - cut.pos_profile_lat[i]) - ego.width / 2 - cut.width / 2 > 0:
        cs = -cut.speed_profile_lat[i]
        if cs > 0:
            hl = (abs(ego.pos_profile_lat[i] - cut.pos_profile_lat[i]) - ego.width / 2 - cut.width / 2) / cs
            rel = ego.speed_profile_long[i] - cut.speed_profile_long[i]
            hg = (abs(ego.pos_profile_long[i] - cut.pos_profile_long[i]) + ego.length / 2 + cut.length / 2) / (rel if abs(rel) > _EPS else _EPS)
            if hl > hg + 0.1: return True
        else:
            return True
    ar = (ego.speed_profile_long[i] - ego.speed_profile_long[i - 1]) * freq
    dist = abs(ego.pos_profile_long[i] - cut.pos_profile_long[i]) - ego.length / 2 - cut.length / 2
    ego.cfs = _CFS(dist, ego.speed_profile_long[i], cut.speed_profile_long[i], ego.FSM_rt, ego.FSM_br_min, ego.FSM_br_max, ego.FSM_bl, ar)
    ego.pfs = _PFS(dist, ego.speed_profile_long[i], cut.speed_profile_long[i], ego.FSM_rt, ego.FSM_br_min, ego.FSM_br_max, ego.FSM_bl, ego.FSM_margin_dist, ego.FSM_margin_safe_dist)
    return True if ego.cfs + ego.pfs == 0 else False


def FSM_react(ego, slog, freq):
    if ego.FSM_rt_counter > 0:
        ego.FSM_rt_counter -= 1 / freq; return slog, 0
    acc = ego.cfs * (ego.FSM_br_max - ego.FSM_br_min) + ego.FSM_br_min if ego.cfs > 0 else ego.pfs * ego.FSM_br_min
    ego.deceleration = min(min(ego.deceleration + ego.CC_min_jerk / freq, ego.FSM_max_deceleration), acc)
    return max(slog - ego.deceleration / freq, 0), 0


MODELS = {"RSS": (RSS_check, RSS_react), "FSM": (FSM_check, FSM_react),
          "CC_human_driver": (CC_check, CC_react), "Reg157": (Reg_check, Reg_react)}


def sim_cut_in(model, ego_ms, obs_ms, v_y, edge_gap, init_lat_c2c=None, it=J_ITER, freq=J_FREQ):
    """Closed-loop cut-in episode. edge_gap = initial EDGE-to-edge longitudinal gap (m),
    init_lat_c2c = initial lateral CENTER-to-center distance (default JRC grid 1.6+W).
    v_y = lateral closing speed magnitude (m/s). Returns True on crash."""
    check, react = MODELS[model]
    lat0 = (1.6 + J_WID) if init_lat_c2c is None else float(init_lat_c2c)
    cut = j_profile_cutting_in(np.array([edge_gap + J_LEN, lat0]), obs_ms, -abs(v_y), it, freq)
    ego = j_profile_cutting_in(np.array([0.0, 0.0]), ego_ms, 0.0, it, freq)
    for i in range(it - 1):
        j_control(ego, cut, freq, check, react, i)
        if ego.crash:
            return True
    return False


def classify_cutin_row(u_ms, w_ms, v_y, p_long_c2c, d_y_c2c, model="CC_human_driver", lon_c2c=J_LEN):
    """Classify one corrected.csv cut-in row with the exact closed-loop model.
    p_long_c2c and d_y_c2c are center-to-center (the schema convention); the
    longitudinal edge gap is recovered by subtracting the summed half lengths.
    Returns 'unsafe' if the model's policy crashes from this state, else 'safe'."""
    edge = max(0.0, float(p_long_c2c) - lon_c2c)
    crash = sim_cut_in(model, float(u_ms), float(w_ms), float(v_y), edge, init_lat_c2c=float(d_y_c2c))
    return "unsafe" if crash else "safe"


# ---------------------------------------------------------------------------
# Closed form of the CC-driver cut-in crash region (continuous-time limit).
# Derivation. Pre-trigger both vehicles hold speed, so with relative speed
# r = u - w > 0 the edge gap is g(t) = d0 - r t and the lateral center gap is
# dy(t) = dy0 - vy t. The trigger needs lateral overlap AND TTC <= 2, both
# monotone once entered, so t_trig = max(t_ov, t_ttc) with
#     t_ov  = (dy0 - W_lat)/vy          (overlap onset)
#     t_ttc = d0/r - 2                  (TTC threshold)
# After the trigger the ego's deceleration profile (coast 0.4 for 0.75 s, jerk
# 12.65 to 7.59, hold) acts directly on r, so the relative distance consumed
# until r = 0 is the CC stopping form applied to r, cc_rel_stop(r) below.
# Crash cases by the gap at overlap onset, G = d0 - r*t_ov (edge gap):
#   G <= -2L                      merged behind, ego clear ahead      -> safe
#   -2L < G < 0                   bodies already overlap at onset    -> crash
#   0 <= G, trigger TTC-limited   gap at trigger = 2r, crash iff cc_rel_stop(r) > 2r
#   0 <= G, trigger overlap-lim.  crash iff cc_rel_stop(r) > G
# which collapses to the band  r*t_ov - 2L  <  d0  <  r*t_ov + C(r),
# with C(r) = cc_rel_stop(r) if cc_rel_stop(r) <= 2r else unbounded above.
# The sim runs at 10 Hz, so its frontier deviates from this limit by up to one
# step of relative travel, about r/f longitudinally.
# ---------------------------------------------------------------------------
CC_TAU, CC_AREL, CC_JERK, CC_AMAX, CC_TTC = 0.75, 0.4, 12.65, 0.774*9.81, 2.0

def cc_rel_stop(r):
    """Relative distance consumed from trigger until the closing speed r reaches 0
    under the CC profile (coast, jerk ramp from 0.4, hold at 7.59)."""
    if r <= 0: return 0.0
    # coast
    t = min(CC_TAU, r/CC_AREL)
    d = r*t - 0.5*CC_AREL*t*t
    r1 = r - CC_AREL*t
    if r1 <= 0: return d
    # jerk ramp, a(tau) = AREL + JERK*tau up to AMAX
    tj = (CC_AMAX - CC_AREL)/CC_JERK
    dv_full = CC_AREL*tj + 0.5*CC_JERK*tj*tj
    if r1 <= dv_full:  # stops inside the ramp; solve AREL*x + J x^2/2 = r1
        x = (-CC_AREL + (CC_AREL**2 + 2*CC_JERK*r1)**0.5)/CC_JERK
        return d + r1*x - (0.5*CC_AREL*x*x + CC_JERK*x**3/6)
    d += r1*tj - (0.5*CC_AREL*tj*tj + CC_JERK*tj**3/6)
    r2 = r1 - dv_full
    return d + r2*r2/(2*CC_AMAX)

def cc_cutin_band(u_ms, w_ms, v_y, d_y_c2c, L=J_LEN, W_lat=J_WID):
    """Closed-form crash band in the initial EDGE gap d0 for the CC cut-in model.
    Returns (lo, hi). Crash iff lo < d0 < hi. hi = inf when even the TTC-limited
    trigger cannot stop in time (cc_rel_stop(r) > 2r). Safe entirely when r<=0."""
    r = u_ms - w_ms
    if r <= 0: return (float('inf'), float('inf'))
    t_ov = max(0.0, (d_y_c2c - W_lat)/max(v_y, 1e-9))
    D = cc_rel_stop(r)
    lo = r*t_ov - 2*L
    hi = r*t_ov + (D if D <= CC_TTC*r else float('inf'))
    return (lo, hi)