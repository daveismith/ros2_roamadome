# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Dome bench analysis: pure functions over lists of tuples, no ROS dependency.

Implements docs/dome_diagnostics_plan.md section 7: unwrap and regression
velocity (7.1), jump, reversal and stall scoring (7.2) and the speed sweep
summary with the per-direction affine fit (7.3), plus the position sweep
(stopping-distance curve and firmware position moves, README "Position-move
tuning") and reading and writing the section 5.5 CSV and the result JSON. The
node in ``node.py`` and the offline ``roamadome_bench_report`` tool both
compute results with :func:`analyze_run_dir`, so a saved run directory
reproduces its JSON exactly.

Conventions:

* ``direction`` is the sign of the commanded percent (+1 or -1), not the sign
  of the angle change. Which way the reported angle moves for a positive
  percent depends on the firmware ``Inverted`` setting and the wiring, so it is
  measured (:func:`angle_sign_mapping`), never assumed.
* Angles are integer degrees as reported by the firmware; times are host
  seconds (the telemetry header stamp).
"""

import collections
import csv
import json
import math
import os

CSV_FIELDS = (
    't_host', 'seq', 'position_deg', 'unwrapped_rad', 'velocity_rad_s',
    'commanded_percent', 'direction', 'step_index', 'phase')

Sample = collections.namedtuple('Sample', CSV_FIELDS)

PHASE_SETTLE = 'settle'
PHASE_MEASURE = 'measure'
# Position sweep phases: steady run before a stop, coasting after it, a firmware move.
PHASE_RUN = 'run'
PHASE_COAST = 'coast'
PHASE_MOVE = 'move'

SAMPLES_CSV = 'samples.csv'
RUN_JSON = 'run.json'
SETTINGS_BEFORE_JSON = 'settings_before.json'
SETTINGS_AFTER_JSON = 'settings_after.json'

DIRECTION_NAMES = {1: 'positive_percent', -1: 'negative_percent'}

# Minimum net displacement for a step to vote on the angle sign mapping (degrees).
SIGN_VOTE_MIN_DEG = 10.0
# Plan 7.2: a jump is |d| > max_plausible_deg_s * dt + JUMP_SLACK_DEG, plus the
# wrap allowance when the step crosses 0: the controller never reports 359 to
# 2 deg, so every 0-crossing step is about 9 deg longer than the dome moved
# (2026-10-16 with-dome report). Absent from an older run.json the allowance is 0.
JUMP_SLACK_DEG = 2.0
# Regression window for the stop-time and settle-time clocks of the position
# sweep: shorter than the velocity window so the standstill is not smeared.
STOP_VELOCITY_WINDOW_S = 0.25
# Speed just before a stop is the slope over this long before the stop command.
PRE_STOP_WINDOW_S = 0.3
# Two report lines read in the same controller cycle are stamped microseconds apart
# (measured 50-80 us), although the firmware sent them a report interval (28 ms) apart.
# Pairs closer than this are "same-cycle" pairs and are not scored for jumps.
SAME_CYCLE_DT_S = 0.005
# Plan 7.2: a reversal needs |d| >= REVERSAL_MIN_DEG for REVERSAL_PAIRS consecutive pairs.
REVERSAL_MIN_DEG = 2
REVERSAL_PAIRS = 2
# Plan 7.3: binary search resolution for min_percent.
SEARCH_RESOLUTION_PERCENT = 2
HISTOGRAM_BIN_DEG = 10

NAN = float('nan')


def _sign(value):
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def _finite(value):
    return value is not None and isinstance(value, (int, float)) and math.isfinite(value)


# ---------------------------------------------------------------------------
# 7.1 unwrap and velocity
# ---------------------------------------------------------------------------

def shortest_delta(from_deg, to_deg):
    """Return the shortest signed step from ``from_deg`` to ``to_deg``, in [-180, 180)."""
    return ((int(to_deg) - int(from_deg) + 180) % 360) - 180


def unwrap_degrees(degrees):
    """Unwrap integer degrees into a continuous angle (same rule as the plugin)."""
    out = []
    total = 0.0
    last = None
    for value in degrees:
        value = int(value) % 360
        if last is None:
            total = float(value)
        else:
            total += shortest_delta(last, value)
        last = value
        out.append(total)
    return out


def regression_slope(times, values):
    """Least-squares slope of ``values`` against ``times``, or NaN if undefined."""
    n = len(times)
    if n < 2 or n != len(values):
        return NAN
    origin = times[0]
    mean_x = sum(t - origin for t in times) / n
    mean_y = sum(values) / n
    sxx = 0.0
    sxy = 0.0
    for t, y in zip(times, values):
        dx = (t - origin) - mean_x
        sxx += dx * dx
        sxy += dx * (y - mean_y)
    if sxx <= 0.0:
        return NAN
    return sxy / sxx


def trailing_velocities(times, values, window_s, min_samples=4, min_span_fraction=0.4):
    """
    Return the regression velocity at every sample over the trailing window.

    Same rule as the plugin's ``PositionTracker``: the window holds the samples
    no older than ``window_s`` before the current one; the slope is NaN unless
    it holds at least ``min_samples`` samples spanning at least
    ``min_span_fraction * window_s``. Units are ``values`` per second.
    """
    out = []
    start = 0
    for i, t in enumerate(times):
        while t - times[start] > window_s:
            start += 1
        count = i - start + 1
        span = t - times[start]
        if count < max(2, min_samples) or span < min_span_fraction * window_s:
            out.append(NAN)
        else:
            out.append(regression_slope(times[start:i + 1], values[start:i + 1]))
    return out


def mean_std(values):
    """Mean and sample standard deviation of the finite values; NaN when undefined."""
    finite = [v for v in values if _finite(v)]
    if not finite:
        return NAN, NAN
    mean = sum(finite) / len(finite)
    if len(finite) < 2:
        return mean, NAN
    var = sum((v - mean) ** 2 for v in finite) / (len(finite) - 1)
    return mean, math.sqrt(var)


# ---------------------------------------------------------------------------
# 7.2 jump, reversal and stall scoring
# ---------------------------------------------------------------------------

class StallDetector:
    """
    Live and offline stall criterion.

    The dome is stalled when, over the last ``stall_s`` seconds or more, its
    average speed (net unwrapped change divided by elapsed time) is below
    ``stall_deg_s``. With no position change at all this is the plan 7.2
    "no change in position for stall_s" rule; the speed threshold also catches
    a dome that only flickers between adjacent readings. Nothing is reported
    until the samples span ``stall_s``.
    """

    def __init__(self, stall_deg_s, stall_s):
        self.stall_deg_s = float(stall_deg_s)
        self.stall_s = float(stall_s)
        self._samples = collections.deque()

    def reset(self):
        self._samples.clear()

    def update(self, t, unwrapped_deg):
        """Feed one sample; return True if the stall criterion holds now."""
        self._samples.append((t, unwrapped_deg))
        cutoff = t - self.stall_s
        # Keep exactly one sample at or before the cutoff as the reference.
        while len(self._samples) >= 2 and self._samples[1][0] <= cutoff:
            self._samples.popleft()
        ref_t, ref_u = self._samples[0]
        if ref_t > cutoff:
            return False
        span = t - ref_t
        return abs(unwrapped_deg - ref_u) < self.stall_deg_s * span


def find_stall(times, unwrapped_deg, stall_deg_s, stall_s):
    """Index of the first sample at which the stall criterion holds, or None."""
    detector = StallDetector(stall_deg_s, stall_s)
    for i, (t, u) in enumerate(zip(times, unwrapped_deg)):
        if detector.update(t, u):
            return i
    return None


def crosses_zero(from_deg, delta_deg):
    """Return True when the shortest step ``delta_deg`` from ``from_deg`` passes through 0."""
    end = int(from_deg) % 360 + int(delta_deg)
    return end < 0 or end >= 360


def is_jump(delta_deg, dt, max_plausible_deg_s, same_cycle_dt_s=SAME_CYCLE_DT_S,
            wrap_allowance_deg=0.0, crossing_zero=False):
    """
    Plan 7.2 jump test for one pair; same-cycle pairs (dt < same_cycle_dt_s) never jump.

    A pair that crosses 0 (``crossing_zero``) is allowed ``wrap_allowance_deg``
    more, for the dead zone the controller skips there.
    """
    if dt < same_cycle_dt_s or dt <= 0.0:
        return False
    allowed = max_plausible_deg_s * dt + JUMP_SLACK_DEG
    if crossing_zero:
        allowed += wrap_allowance_deg
    return abs(delta_deg) > allowed


def score_jumps(times, degrees, max_plausible_deg_s, percents=None,
                same_cycle_dt_s=SAME_CYCLE_DT_S, wrap_allowance_deg=0.0):
    """
    Score every consecutive pair for implausible jumps.

    Pairs read in the same controller cycle (dt below ``same_cycle_dt_s``,
    including dt == 0) have no usable interval: they are skipped and counted
    separately. Returns ``(jumps, same_cycle_pairs, jump_pair_indices)`` where
    each jump is a dict with the start angle, change, interval and commanded
    percent.
    """
    jumps = []
    same_cycle = 0
    indices = set()
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        d = shortest_delta(degrees[i - 1], degrees[i])
        if dt < same_cycle_dt_s or dt <= 0.0:
            same_cycle += 1
            continue
        if is_jump(d, dt, max_plausible_deg_s, same_cycle_dt_s, wrap_allowance_deg,
                   crosses_zero(degrees[i - 1], d)):
            indices.add(i)
            jumps.append({
                't_host': times[i - 1],
                'start_deg': int(degrees[i - 1]) % 360,
                'delta_deg': d,
                'dt_s': dt,
                'commanded_percent': None if percents is None else int(percents[i]),
            })
    return jumps, same_cycle, indices


def score_reversals(degrees, expected_sign, exclude=frozenset()):
    """
    Count reversal events against the expected direction of angle change.

    A reversal is ``sign(d)`` opposite to ``expected_sign`` with
    ``|d| >= REVERSAL_MIN_DEG`` for ``REVERSAL_PAIRS`` consecutive pairs; a run
    of backward pairs counts once. Pairs in ``exclude`` (scored as jumps) break
    a run. Returns ``(events, backward_pairs)``.
    """
    if expected_sign == 0:
        return 0, 0
    events = 0
    backward = 0
    run = 0
    for i in range(1, len(degrees)):
        d = shortest_delta(degrees[i - 1], degrees[i])
        if i not in exclude and abs(d) >= REVERSAL_MIN_DEG and _sign(d) == -expected_sign:
            backward += 1
            run += 1
            if run == REVERSAL_PAIRS:
                events += 1
        else:
            run = 0
    return events, backward


def max_gap(times):
    """Return the largest interval between consecutive samples (0 for fewer than 2)."""
    return max((b - a for a, b in zip(times, times[1:])), default=0.0)


def jump_histogram(jumps, bin_deg=HISTOGRAM_BIN_DEG):
    """Histogram of jump start angles in ``bin_deg`` bins over 0..359."""
    bins = [0] * (360 // bin_deg)
    for jump in jumps:
        bins[(int(jump['start_deg']) % 360) // bin_deg] += 1
    return bins


# ---------------------------------------------------------------------------
# 7.3 speed sweep helpers
# ---------------------------------------------------------------------------

def measure_duration(speed_deg_s, min_measure_s, max_measure_s):
    """Plan 7.3 measure window: max(min, one revolution at the speed), capped."""
    duration = float(min_measure_s)
    if _finite(speed_deg_s) and abs(speed_deg_s) > 0.0:
        duration = max(duration, 360.0 / abs(speed_deg_s))
    return min(duration, float(max_measure_s))


def fit_affine(percents, speeds):
    """
    Least-squares fit ``speed = k * (p - p0)``.

    Returns a dict with ``k`` (speed units per percent), ``p0``, ``r_squared``
    and ``points``, or None with fewer than two distinct percents or a zero
    slope.
    """
    n = len(percents)
    if n < 2 or len(set(percents)) < 2:
        return None
    mean_p = sum(percents) / n
    mean_s = sum(speeds) / n
    spp = sum((p - mean_p) ** 2 for p in percents)
    sps = sum((p - mean_p) * (s - mean_s) for p, s in zip(percents, speeds))
    k = sps / spp
    if k == 0.0:
        return None
    intercept = mean_s - k * mean_p
    p0 = -intercept / k
    ss_res = sum((s - (intercept + k * p)) ** 2 for p, s in zip(percents, speeds))
    ss_tot = sum((s - mean_s) ** 2 for s in speeds)
    if ss_tot > 0.0:
        r_squared = 1.0 - ss_res / ss_tot
    else:
        r_squared = 1.0 if ss_res == 0.0 else NAN
    return {'k': k, 'p0': p0, 'r_squared': r_squared, 'points': n}


def min_percent_bracket(entries):
    """
    Return ``(highest_stalled, lowest_moving)`` for one direction.

    ``entries`` are ``(abs_percent, moving)`` pairs; ``highest_stalled`` is the
    highest stalled percent below the lowest moving one. Either is None when
    absent.
    """
    moving = [p for p, ok in entries if ok]
    lowest_moving = min(moving) if moving else None
    stalled = [p for p, ok in entries if not ok and
               (lowest_moving is None or p < lowest_moving)]
    highest_stalled = max(stalled) if stalled else None
    return highest_stalled, lowest_moving


def next_search_probe(entries, resolution=SEARCH_RESOLUTION_PERCENT):
    """Next percent to probe in the min_percent binary search, or None when done."""
    low, high = min_percent_bracket(entries)
    if low is None or high is None or high - low <= resolution:
        return None
    return (low + high) // 2


# ---------------------------------------------------------------------------
# Steps and sweeps
# ---------------------------------------------------------------------------

def group_steps(rows):
    """Group rows by ``step_index`` preserving order."""
    groups = collections.OrderedDict()
    for row in rows:
        groups.setdefault(int(row.step_index), []).append(row)
    return groups


def _derive_step_meta(index, rows):
    percents = [int(r.commanded_percent) for r in rows if int(r.commanded_percent) != 0]
    if percents:
        percent = collections.Counter(percents).most_common(1)[0][0]
    else:
        percent = 0
    direction = int(rows[0].direction) if rows else _sign(percent)
    return {'index': index, 'kind': 'step', 'percent': percent, 'direction': direction}


def summarize_step(rows, meta, config):
    """
    Per-step statistics shared by both sweeps.

    Jumps, the stall criterion and the gap statistics use every row of the
    step (settle and measure); velocity statistics use the measure rows only.
    """
    times = [float(r.t_host) for r in rows]
    degrees = [int(r.position_deg) for r in rows]
    unwrapped = unwrap_degrees(degrees)
    window_s = float(config.get('velocity_window_s', 0.5))
    velocities = trailing_velocities(times, unwrapped, window_s)
    measure_idx = [i for i, r in enumerate(rows) if r.phase == PHASE_MEASURE]

    stall_index = find_stall(times, unwrapped, config['stall_deg_s'], config['stall_s'])
    jumps, same_cycle, jump_pairs = score_jumps(
        times, degrees, config['max_plausible_deg_s'],
        [int(r.commanded_percent) for r in rows],
        float(config.get('same_cycle_dt_s', SAME_CYCLE_DT_S)),
        float(config.get('wrap_allowance_deg', 0.0)))

    summary = collections.OrderedDict()
    summary['index'] = int(meta['index'])
    summary['kind'] = meta.get('kind', 'step')
    summary['percent'] = int(meta['percent'])
    summary['direction'] = int(meta['direction'])
    summary['samples'] = len(rows)
    summary['measure_samples'] = len(measure_idx)
    summary['stalled'] = stall_index is not None
    summary['stall_after_s'] = (times[stall_index] - times[0]) if stall_index is not None else None
    summary['end_reason'] = meta.get('end_reason')
    summary['jump_count'] = len(jumps)
    summary['same_cycle_pairs'] = same_cycle
    summary['max_gap_s'] = max_gap(times)
    summary['step_displacement_deg'] = (unwrapped[-1] - unwrapped[0]) if rows else 0.0

    if len(measure_idx) >= 2:
        m_times = [times[i] for i in measure_idx]
        m_unwrapped = [unwrapped[i] for i in measure_idx]
        slope_deg_s = regression_slope(m_times, m_unwrapped)
        mean_v, std_v = mean_std([velocities[i] for i in measure_idx])
        displacement = m_unwrapped[-1] - m_unwrapped[0]
        summary['measure_s'] = m_times[-1] - m_times[0]
        summary['displacement_deg'] = displacement
        summary['revolutions'] = abs(displacement) / 360.0
        summary['velocity_rad_s'] = math.radians(slope_deg_s) if _finite(slope_deg_s) else NAN
        summary['mean_velocity_rad_s'] = math.radians(mean_v) if _finite(mean_v) else NAN
        summary['std_velocity_rad_s'] = math.radians(std_v) if _finite(std_v) else NAN
    else:
        summary['measure_s'] = 0.0
        summary['displacement_deg'] = 0.0
        summary['revolutions'] = 0.0
        summary['velocity_rad_s'] = NAN
        summary['mean_velocity_rad_s'] = NAN
        summary['std_velocity_rad_s'] = NAN
    summary['speed_rad_s'] = (abs(summary['velocity_rad_s'])
                              if _finite(summary['velocity_rad_s']) else NAN)
    # Moving: completed a measure window without stalling.
    summary['moving'] = (not summary['stalled'] and len(measure_idx) >= 2 and
                         _finite(summary['velocity_rad_s']))
    summary['jumps'] = jumps
    return summary, degrees, jump_pairs


def angle_sign_mapping(summaries):
    """
    Measured sign of the angle change for a positive percent.

    Each step that moved at least ``SIGN_VOTE_MIN_DEG`` votes
    ``sign(displacement) * direction``. Returns ``(sign, consistent, votes)``
    with ``sign`` 0 if there are no votes or they tie.
    """
    votes = {1: 0, -1: 0}
    for s in summaries:
        disp = s['step_displacement_deg']
        if s['direction'] != 0 and abs(disp) >= SIGN_VOTE_MIN_DEG:
            votes[_sign(disp) * s['direction']] += 1
    if votes[1] == votes[-1]:
        sign = 0
    else:
        sign = 1 if votes[1] > votes[-1] else -1
    consistent = votes[1] == 0 or votes[-1] == 0
    return sign, consistent, {'positive': votes[1], 'negative': votes[-1]}


def _steps_meta(run_info, groups):
    metas = collections.OrderedDict()
    for meta in run_info.get('steps', []):
        metas[int(meta['index'])] = meta
    for index, rows in groups.items():
        if index not in metas:
            metas[index] = _derive_step_meta(index, rows)
    return metas


def _min_known_for(config, direction):
    known = config.get('min_percent_known') or {}
    value = known.get(DIRECTION_NAMES[direction]) if isinstance(known, dict) else known
    return int(value) if _finite(value) else 0


def _strip(summary, keep_jumps):
    out = collections.OrderedDict(summary)
    if not keep_jumps:
        out.pop('jumps', None)
    return out


def _header(run_info):
    header = collections.OrderedDict()
    header['sweep'] = run_info.get('sweep')
    header['run_id'] = run_info.get('run_id')
    header['status'] = run_info.get('status')
    header['partial'] = run_info.get('status') != 'complete'
    header['reason'] = run_info.get('reason')
    header['config'] = run_info.get('config', {})
    return header


def analyze_sensor_sweep(run_info, rows):
    """Plan 7.2 sensor sweep result."""
    config = run_info.get('config', {})
    groups = group_steps(rows)
    metas = _steps_meta(run_info, groups)
    summaries = []
    degrees_by_step = {}
    for index, meta in metas.items():
        step_rows = groups.get(index, [])
        summary, degrees, jump_pairs = summarize_step(step_rows, meta, config)
        summaries.append(summary)
        degrees_by_step[index] = (degrees, jump_pairs)

    sign, consistent, votes = angle_sign_mapping(summaries)
    all_jumps = []
    steps = []
    for summary in summaries:
        degrees, jump_pairs = degrees_by_step[summary['index']]
        gated = abs(summary['percent']) >= _min_known_for(config, summary['direction'])
        expected = sign * summary['direction']
        if gated and expected != 0:
            reversals, backward = score_reversals(degrees, expected, jump_pairs)
        else:
            reversals, backward = 0, 0
        summary['reversal_count'] = reversals
        summary['backward_pairs'] = backward
        summary['stall_count'] = 1 if (summary['stalled'] and gated) else 0
        summary['scored_reversals_and_stalls'] = gated and expected != 0
        all_jumps.extend(summary['jumps'])
        steps.append(_strip(summary, keep_jumps=True))

    result = _header(run_info)
    result['angle_sign_for_positive_percent'] = sign
    result['angle_sign_consistent'] = consistent
    result['angle_sign_votes'] = votes
    result['steps'] = steps
    result['jump_start_histogram'] = {
        'bin_deg': HISTOGRAM_BIN_DEG,
        'counts': jump_histogram(all_jumps),
    }
    result['totals'] = {
        'samples': sum(s['samples'] for s in steps),
        'jumps': sum(s['jump_count'] for s in steps),
        'reversals': sum(s['reversal_count'] for s in steps),
        'stalls': sum(s['stall_count'] for s in steps),
        'same_cycle_pairs': sum(s['same_cycle_pairs'] for s in steps),
        'max_gap_s': max((s['max_gap_s'] for s in steps), default=0.0),
    }
    # The plugin's plausibility gate (max_plausible_deg_s) drops a misread before
    # it reaches telemetry, so its counters over the sweep are part of the verdict.
    gate = gate_counters_delta(run_info.get('sensor_diagnostics_before'),
                               run_info.get('sensor_diagnostics_after'))
    result['plugin_gate'] = gate
    result['suggested_pass'] = (bool(steps) and result['totals']['jumps'] == 0 and
                                result['totals']['reversals'] == 0 and
                                (gate['reports_rejected'] or 0) == 0 and
                                (gate['gate_resyncs'] or 0) == 0)
    return result


GATE_COUNTERS = ('reports_held', 'reports_rejected', 'jumps_confirmed', 'gate_resyncs')


def gate_counters_delta(before, after):
    """
    Change in the plugin's plausibility-gate counters over a sweep.

    ``before`` and ``after`` are the ``roamadome: sensor`` value dicts the node
    saved; a counter missing on either side (older plugin, or no diagnostics
    seen) gives None. ``known`` is False when nothing could be read.
    """
    out = collections.OrderedDict()
    known = False
    for key in GATE_COUNTERS:
        try:
            out[key] = int((after or {})[key]) - int((before or {})[key])
            known = True
        except (KeyError, TypeError, ValueError):
            out[key] = None
    out['known'] = known
    return out


def _direction_fit(summaries, direction, angle_sign):
    steps = [s for s in summaries if s['direction'] == direction]
    entries = [(abs(s['percent']), s['moving']) for s in steps
               if s['moving'] or s['stalled']]
    highest_stalled, lowest_moving = min_percent_bracket(entries)
    moving = [s for s in steps if s['moving']]
    out = collections.OrderedDict()
    out['angle_sign'] = angle_sign * direction if angle_sign else 0
    out['moving_steps'] = len(moving)
    out['stalled_steps'] = sum(1 for s in steps if s['stalled'])
    out['highest_stalled_percent'] = highest_stalled
    out['lowest_moving_percent'] = lowest_moving
    out['min_percent'] = lowest_moving
    out['min_percent_bracketed'] = (
        highest_stalled is not None and lowest_moving is not None and
        lowest_moving - highest_stalled <= SEARCH_RESOLUTION_PERCENT)
    at_min = [s['speed_rad_s'] for s in moving if abs(s['percent']) == lowest_moving]
    out['velocity_at_min_percent_rad_s'] = (sum(at_min) / len(at_min)) if at_min else None
    out['stalled_above_min_percent'] = sorted(
        abs(s['percent']) for s in steps
        if s['stalled'] and lowest_moving is not None and abs(s['percent']) > lowest_moving)
    wrong_way = [s['index'] for s in moving
                 if angle_sign and _sign(s['velocity_rad_s']) not in (0, angle_sign * direction)]
    out['wrong_direction_steps'] = wrong_way

    fit = fit_affine([abs(s['percent']) for s in moving], [s['speed_rad_s'] for s in moving])
    at_100 = [s['speed_rad_s'] for s in moving if abs(s['percent']) == 100]
    out['omega_max_measured_rad_s'] = (sum(at_100) / len(at_100)) if at_100 else None
    out['max_measured_speed_rad_s'] = max((s['speed_rad_s'] for s in moving), default=None)
    out['highest_moving_percent'] = max((abs(s['percent']) for s in moving), default=None)
    if fit is None:
        out['fit'] = None
        out['omega_max_rad_s'] = out['omega_max_measured_rad_s']
    else:
        out['fit'] = collections.OrderedDict([
            ('k_rad_s_per_percent', fit['k']),
            ('p0_percent', fit['p0']),
            ('r_squared', fit['r_squared']),
            ('points', fit['points']),
        ])
        out['omega_max_rad_s'] = fit['k'] * (100.0 - fit['p0'])
    return out


def analyze_speed_sweep(run_info, rows):
    """Plan 7.3 speed sweep result."""
    config = run_info.get('config', {})
    groups = group_steps(rows)
    metas = _steps_meta(run_info, groups)
    summaries = []
    for index, meta in metas.items():
        summary, _, _ = summarize_step(groups.get(index, []), meta, config)
        summaries.append(summary)

    sign, consistent, votes = angle_sign_mapping(summaries)
    directions = collections.OrderedDict()
    for direction in (1, -1):
        directions[DIRECTION_NAMES[direction]] = _direction_fit(summaries, direction, sign)

    pos = directions['positive_percent']['omega_max_rad_s']
    neg = directions['negative_percent']['omega_max_rad_s']
    asymmetry = (pos / neg) if (_finite(pos) and _finite(neg) and neg != 0.0) else None
    maxima = [v for v in (pos, neg) if _finite(v)]
    recommended = min(maxima) if maxima else None

    phase7 = collections.OrderedDict()
    for velocity_sign, name in ((1, 'positive_velocity'), (-1, 'negative_velocity')):
        entry = collections.OrderedDict()
        percent_sign = velocity_sign * sign if sign else 0
        entry['percent_sign'] = percent_sign
        fit_dir = directions.get(DIRECTION_NAMES.get(percent_sign, ''), None)
        fit = fit_dir['fit'] if fit_dir else None
        entry['k_percent_per_rad_s'] = (1.0 / fit['k_rad_s_per_percent']) if fit else None
        entry['p0_percent'] = fit['p0_percent'] if fit else None
        entry['min_percent'] = fit_dir['min_percent'] if fit_dir else None
        phase7[name] = entry
    phase7['max_speed_rad_per_sec'] = recommended

    moving = [s for s in summaries if s['moving']]
    result = _header(run_info)
    result['angle_sign_for_positive_percent'] = sign
    result['angle_sign_consistent'] = consistent
    result['angle_sign_votes'] = votes
    result['steps'] = [_strip(s, keep_jumps=False) for s in summaries]
    result['directions'] = directions
    result['asymmetry_ratio'] = asymmetry
    result['recommended_max_speed_rad_per_sec'] = recommended
    result['max_measured_speed_rad_s'] = max((s['speed_rad_s'] for s in moving), default=None)
    result['highest_moving_percent'] = max((abs(s['percent']) for s in moving), default=None)
    result['phase7'] = phase7
    result['totals'] = {
        'samples': sum(s['samples'] for s in summaries),
        'jumps': sum(s['jump_count'] for s in summaries),
        'stalled_steps': sum(1 for s in summaries if s['stalled']),
        'same_cycle_pairs': sum(s['same_cycle_pairs'] for s in summaries),
        'max_gap_s': max((s['max_gap_s'] for s in summaries), default=0.0),
    }
    return result


# ---------------------------------------------------------------------------
# Position sweep: stopping-distance curve and firmware position moves
# ---------------------------------------------------------------------------

def fit_line(xs, ys):
    """Least-squares ``y = slope * x + intercept``; None with fewer than two distinct x."""
    n = len(xs)
    if n < 2 or len(set(xs)) < 2 or n != len(ys):
        return None
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    sxx = sum((x - mean_x) ** 2 for x in xs)
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    slope = sxy / sxx
    intercept = mean_y - slope * mean_x
    ss_res = sum((y - (intercept + slope * x)) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - mean_y) ** 2 for y in ys)
    if ss_tot > 0.0:
        r_squared = 1.0 - ss_res / ss_tot
    else:
        r_squared = 1.0 if ss_res == 0.0 else NAN
    return {'slope': slope, 'intercept': intercept, 'r_squared': r_squared, 'points': n}


def _last_moving_time(times, velocities, stopped_deg_s, after_t):
    """Latest sample time at or after ``after_t`` whose speed exceeds ``stopped_deg_s``."""
    last = None
    for t, v in zip(times, velocities):
        if t >= after_t and _finite(v) and abs(v) > stopped_deg_s:
            last = t
    return last


def summarize_stop_step(rows, meta, config):
    """
    One stopping-distance step: steady percent, then 0, coasting to standstill.

    The stop instant is the first telemetry sample that reports
    ``commanded_percent`` 0 after the drive (the plugin stamps the percent it
    last sent, so this is within one report of the ``:DPR0``); the speed just
    before it is the slope over the previous ``PRE_STOP_WINDOW_S``; the stop
    distance is the unwrapped travel from that instant to the last sample.
    """
    times = [float(r.t_host) for r in rows]
    degrees = [int(r.position_deg) for r in rows]
    unwrapped = unwrap_degrees(degrees)
    stopped_deg_s = float(config.get('stopped_deg_s', 5.0))
    jumps, same_cycle, _ = score_jumps(
        times, degrees, config.get('max_plausible_deg_s', 720.0),
        [int(r.commanded_percent) for r in rows],
        float(config.get('same_cycle_dt_s', SAME_CYCLE_DT_S)),
        float(config.get('wrap_allowance_deg', 0.0)))

    summary = collections.OrderedDict()
    summary['index'] = int(meta['index'])
    summary['kind'] = 'stop'
    summary['percent'] = int(meta['percent'])
    summary['direction'] = int(meta['direction'])
    summary['samples'] = len(rows)
    summary['end_reason'] = meta.get('end_reason')
    summary['settled'] = meta.get('settled')
    summary['jump_count'] = len(jumps)
    summary['same_cycle_pairs'] = same_cycle
    summary['max_gap_s'] = max_gap(times)

    i_stop = None
    driven = False
    for i, r in enumerate(rows):
        if int(r.commanded_percent) != 0:
            driven = True
        elif driven:
            i_stop = i
            break
    if i_stop is None:
        coast = [i for i, r in enumerate(rows) if r.phase == PHASE_COAST]
        i_stop = coast[0] if coast else None
    if i_stop is None or i_stop < 1:
        summary['stop_found'] = False
        summary['speed_before_stop_deg_s'] = NAN
        summary['speed_before_stop_rad_s'] = NAN
        summary['stop_distance_deg'] = NAN
        summary['stop_time_s'] = NAN
        return summary
    t_stop = times[i_stop]
    pre = [i for i in range(i_stop) if times[i] >= t_stop - PRE_STOP_WINDOW_S]
    if len(pre) >= 3:
        speed = regression_slope([times[i] for i in pre], [unwrapped[i] for i in pre])
    else:
        speed = NAN
    velocities = trailing_velocities(times, unwrapped, STOP_VELOCITY_WINDOW_S)
    last_moving = _last_moving_time(times, velocities, stopped_deg_s, t_stop)
    # A pass through 0 deg adds the dead zone the controller skips there (about
    # 9 deg) to the apparent travel: flag it for the speed window and the coast.
    summary['pre_stop_crosses_zero'] = any(
        crosses_zero(degrees[i - 1], shortest_delta(degrees[i - 1], degrees[i]))
        for i in pre[1:] + [i_stop] if i >= 1)
    summary['coast_crosses_zero'] = any(
        crosses_zero(degrees[i - 1], shortest_delta(degrees[i - 1], degrees[i]))
        for i in range(i_stop + 1, len(degrees)))
    summary['stop_found'] = True
    summary['t_stop'] = t_stop
    summary['speed_before_stop_deg_s'] = abs(speed) if _finite(speed) else NAN
    summary['speed_before_stop_rad_s'] = math.radians(abs(speed)) if _finite(speed) else NAN
    summary['stop_distance_deg'] = abs(unwrapped[-1] - unwrapped[i_stop])
    summary['stop_time_s'] = (last_moving - t_stop) if last_moving is not None else 0.0
    return summary


def summarize_move_step(rows, meta, config):
    """
    Summarise one firmware position move (``:DPA``).

    Peak speed, overshoot, rest error, settle time and, when the fudge is
    known, the speed on entering the fudge band.
    """
    times = [float(r.t_host) for r in rows]
    degrees = [int(r.position_deg) for r in rows]
    unwrapped = unwrap_degrees(degrees)
    target = int(meta['target_deg'])
    delta = int(meta['delta_deg'])
    direction = _sign(delta)
    stopped_deg_s = float(config.get('stopped_deg_s', 5.0))
    fudge = config.get('fudge_deg')
    t_cmd = float(meta.get('t_move_wall', times[0] if times else 0.0))
    velocities = trailing_velocities(times, unwrapped, STOP_VELOCITY_WINDOW_S)
    jumps, same_cycle, _ = score_jumps(
        times, degrees, config.get('max_plausible_deg_s', 720.0),
        [int(r.commanded_percent) for r in rows],
        float(config.get('same_cycle_dt_s', SAME_CYCLE_DT_S)),
        float(config.get('wrap_allowance_deg', 0.0)))

    summary = collections.OrderedDict()
    summary['index'] = int(meta['index'])
    summary['kind'] = 'move'
    summary['percent'] = 0
    summary['direction'] = direction
    summary['start_deg'] = meta.get('start_deg')
    summary['target_deg'] = target
    summary['delta_deg'] = delta
    summary['speed_percent'] = meta.get('speed_percent', 0)
    # The controller never reports 356..3 deg, so a rest error against a target
    # there is inflated by the dead zone.
    summary['target_in_dead_zone'] = target >= 356 or target <= 3
    summary['samples'] = len(rows)
    summary['end_reason'] = meta.get('end_reason')
    summary['settled'] = meta.get('settled')
    summary['firmware'] = meta.get('firmware')
    summary['jump_count'] = len(jumps)
    summary['same_cycle_pairs'] = same_cycle
    summary['max_gap_s'] = max_gap(times)
    if not rows:
        for key in ('peak_deg_s', 'furthest_past_target_deg', 'rest_error_deg', 'settle_s',
                    'arrival_deg_s'):
            summary[key] = NAN
        summary['reached_target'] = False
        return summary
    finite_v = [abs(v) for v in velocities if _finite(v)]
    summary['peak_deg_s'] = max(finite_v) if finite_v else NAN
    # Signed distance past the target along the direction of travel: positive is
    # overshoot, negative means the dome never reached the target.
    past = [shortest_delta(target, d) * direction for d in degrees]
    summary['furthest_past_target_deg'] = max(past)
    summary['reached_target'] = max(past) >= 0
    summary['rest_error_deg'] = shortest_delta(target, degrees[-1])
    last_moving = _last_moving_time(times, velocities, stopped_deg_s, t_cmd)
    summary['settle_s'] = (last_moving - t_cmd) if last_moving is not None else 0.0
    arrival = NAN
    if _finite(fudge):
        for i, d in enumerate(degrees):
            if abs(shortest_delta(target, d)) <= fudge and times[i] >= t_cmd:
                arrival = abs(velocities[i]) if _finite(velocities[i]) else NAN
                break
    summary['arrival_deg_s'] = arrival
    summary['jumps'] = jumps
    return summary


def _abs_stats(values):
    finite = [abs(v) for v in values if _finite(v)]
    if not finite:
        return {'count': 0, 'max_abs': None, 'mean_abs': None}
    return {'count': len(finite), 'max_abs': max(finite), 'mean_abs': sum(finite) / len(finite)}


def analyze_position_sweep(run_info, rows):
    """Position sweep result: the stopping-distance curve and the move table."""
    config = dict(run_info.get('config', {}))
    settings = run_info.get('settings_context') or {}
    if 'fudge_deg' not in config and _finite(settings.get('fudge')):
        config['fudge_deg'] = int(settings['fudge'])
    groups = group_steps(rows)
    metas = _steps_meta(run_info, groups)

    stops = []
    moves = []
    for index, meta in metas.items():
        step_rows = groups.get(index, [])
        if meta.get('kind') == 'move':
            moves.append(summarize_move_step(step_rows, meta, config))
        else:
            stops.append(summarize_stop_step(step_rows, meta, config))

    result = _header(run_info)
    result['settings'] = settings
    result['move_speed_percent'] = int(config.get('move_speed_percent', 0) or 0)

    curve = collections.OrderedDict()
    # Only steps that ran their full time and coasted to rest carry a valid distance.
    usable = [s for s in stops if s.get('stop_found') and _finite(s['speed_before_stop_deg_s'])
              and s.get('end_reason') == 'completed' and s.get('settled') is True]
    curve['steps'] = stops
    curve['fit_steps'] = [s['index'] for s in usable]
    curve['fit_steps_crossing_zero'] = [s['index'] for s in usable if
                                        s.get('pre_stop_crosses_zero') or
                                        s.get('coast_crosses_zero')]
    curve['fit'] = fit_line([s['speed_before_stop_deg_s'] for s in usable],
                            [s['stop_distance_deg'] for s in usable])
    for direction in (1, -1):
        subset = [s for s in usable if s['direction'] == direction]
        curve['fit_' + DIRECTION_NAMES[direction]] = fit_line(
            [s['speed_before_stop_deg_s'] for s in subset],
            [s['stop_distance_deg'] for s in subset])
    if curve['fit']:
        curve['fit']['note'] = ('stop_distance_deg = slope [s] x speed_before_stop_deg_s + '
                                'intercept [deg]')
    result['stop_curve'] = curve

    move_table = collections.OrderedDict()
    move_table['steps'] = [_strip(m, keep_jumps=False) for m in moves]
    completed = [m for m in moves if m.get('end_reason') == 'completed']
    move_table['count'] = len(moves)
    move_table['completed'] = len(completed)
    move_table['timed_out'] = sum(1 for m in moves if m.get('end_reason') == 'timeout')
    move_table['never_reached_target'] = sum(1 for m in moves if not m.get('reached_target'))
    move_table['firmware_timeouts'] = sum(
        int((m.get('firmware') or {}).get('position_timeouts', 0) or 0) for m in moves)
    move_table['rest_error_deg'] = _abs_stats([m['rest_error_deg'] for m in completed])
    for direction, name in ((1, 'angle_increasing'), (-1, 'angle_decreasing')):
        subset = [m['rest_error_deg'] for m in completed if m['direction'] == direction]
        move_table['rest_error_deg_' + name] = _abs_stats(subset)
    overs = [m['furthest_past_target_deg'] for m in completed
             if _finite(m['furthest_past_target_deg']) and m['furthest_past_target_deg'] > 0]
    move_table['overshoot_max_deg'] = max(overs) if overs else 0
    arrivals = [m['arrival_deg_s'] for m in completed if _finite(m['arrival_deg_s'])]
    move_table['arrival_deg_s'] = {
        'count': len(arrivals),
        'min': min(arrivals) if arrivals else None,
        'max': max(arrivals) if arrivals else None,
        'mean': (sum(arrivals) / len(arrivals)) if arrivals else None,
    }
    if curve['fit'] and arrivals:
        fit = curve['fit']
        move_table['coast_from_arrival_deg'] = {
            'min': fit['slope'] * min(arrivals) + fit['intercept'],
            'max': fit['slope'] * max(arrivals) + fit['intercept'],
        }
    result['moves'] = move_table
    result['totals'] = {
        'samples': sum(s['samples'] for s in stops) + sum(m['samples'] for m in moves),
        'jumps': sum(s['jump_count'] for s in stops) + sum(m['jump_count'] for m in moves),
        'same_cycle_pairs': (sum(s['same_cycle_pairs'] for s in stops) +
                             sum(m['same_cycle_pairs'] for m in moves)),
        'max_gap_s': max([s['max_gap_s'] for s in stops] + [m['max_gap_s'] for m in moves],
                         default=0.0),
    }
    return result


def analyze_run(run_info, rows):
    """Dispatch on ``run_info['sweep']``."""
    sweep = run_info.get('sweep')
    if sweep == 'sensor':
        return analyze_sensor_sweep(run_info, rows)
    if sweep == 'speed':
        return analyze_speed_sweep(run_info, rows)
    if sweep == 'position':
        return analyze_position_sweep(run_info, rows)
    raise ValueError('unknown sweep type: %r' % (sweep,))


def result_filename(sweep):
    """Return the result JSON file name for a sweep type."""
    return '%s_sweep.json' % sweep


def analyze_run_dir(run_dir):
    """Re-run the analysis on a run directory (``run.json`` + ``samples.csv``)."""
    run_info = read_json(os.path.join(run_dir, RUN_JSON))
    rows = read_csv(os.path.join(run_dir, SAMPLES_CSV))
    return analyze_run(run_info, rows)


def sensor_thresholds_from_speed_result(result, raw_motor, highest_percent):
    """
    Thresholds a sensor sweep may take from an earlier speed sweep result.

    Plan 7.2: ``max_plausible_deg_s`` becomes 1.5 times the measured maximum
    once a speed sweep result exists. It is only used when that sweep ran with
    the same ``raw_motor`` setting, completed, and moved at or above
    ``highest_percent`` (so its maximum covers the sensor sweep's speeds).
    Returns ``(max_plausible_deg_s, min_percent_known)`` or None.
    """
    if not result or result.get('status') != 'complete':
        return None
    if bool(result.get('config', {}).get('raw_motor')) != bool(raw_motor):
        return None
    top = result.get('highest_moving_percent')
    speed = result.get('max_measured_speed_rad_s')
    if not _finite(top) or not _finite(speed) or top < highest_percent:
        return None
    min_known = {}
    for name in DIRECTION_NAMES.values():
        value = result.get('directions', {}).get(name, {}).get('min_percent')
        min_known[name] = value
    return 1.5 * math.degrees(speed), min_known


# ---------------------------------------------------------------------------
# CSV and JSON
# ---------------------------------------------------------------------------

def format_row(sample):
    """CSV cells for one sample; floats use repr so they read back exactly."""
    return [repr(float(sample.t_host)), str(int(sample.seq)), str(int(sample.position_deg)),
            repr(float(sample.unwrapped_rad)), repr(float(sample.velocity_rad_s)),
            str(int(sample.commanded_percent)), str(int(sample.direction)),
            str(int(sample.step_index)), str(sample.phase)]


class CsvSampleWriter:
    """Append samples to a 5.5 schema CSV, flushing after every batch."""

    def __init__(self, path):
        self.path = path
        self._file = open(path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(CSV_FIELDS)
        self._file.flush()

    def write(self, samples):
        for sample in samples:
            self._writer.writerow(format_row(sample))
        self._file.flush()

    def close(self):
        if not self._file.closed:
            self._file.close()


def write_csv(path, samples):
    """Write a complete CSV."""
    writer = CsvSampleWriter(path)
    try:
        writer.write(samples)
    finally:
        writer.close()


def parse_row(record):
    """Return a typed :class:`Sample` from a CSV dict record."""
    return Sample(
        t_host=float(record['t_host']),
        seq=int(record['seq']),
        position_deg=int(record['position_deg']),
        unwrapped_rad=float(record['unwrapped_rad']),
        velocity_rad_s=float(record['velocity_rad_s']),
        commanded_percent=int(record['commanded_percent']),
        direction=int(record['direction']),
        step_index=int(record['step_index']),
        phase=record['phase'],
    )


def read_csv(path):
    """Read a 5.5 schema CSV into a list of :class:`Sample`; skips a truncated last line."""
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        missing = [name for name in CSV_FIELDS if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError('%s: missing CSV columns %s' % (path, ', '.join(missing)))
        for record in reader:
            try:
                rows.append(parse_row(record))
            except (TypeError, ValueError):
                continue  # a partial last line after SIGKILL
    return rows


def to_jsonable(value):
    """Replace non-finite floats with None, recursively, so the JSON is strict."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return collections.OrderedDict((k, to_jsonable(v)) for k, v in value.items())
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    return value


def dumps_json(value):
    """Return strict JSON text (no NaN), two-space indented, with a trailing newline."""
    return json.dumps(to_jsonable(value), indent=2, allow_nan=False) + '\n'


def write_json(path, value):
    """Write ``value`` as strict JSON atomically (write then rename)."""
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write(dumps_json(value))
    os.replace(tmp, path)


def read_json(path):
    """Read a JSON file preserving key order."""
    with open(path) as f:
        return json.load(f, object_pairs_hook=collections.OrderedDict)
